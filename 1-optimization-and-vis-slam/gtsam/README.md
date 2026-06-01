---
topic: GTSAM factor graph optimization
tags: [gtsam, slam, factor graph, optimization, nuscenes, mcap, lidar, extrinsics, robust]
scope: Local to gtsam optimization
---

# NuFuse — Multi-Sensor Factor Graph Optimizer

A tightly-integrated SLAM back-end built on [GTSAM](https://gtsam.org/) that fuses IMU preintegration, GNSS fixes, and LiDAR scan-matching odometry into a single factor graph. It jointly optimizes the vehicle trajectory **and** the LiDAR extrinsic calibration, using robust factors to handle GPS outliers gracefully.

## Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc)
```

```bash
./build/nufuse ../../data/nuscenes/mcap/NuScenes-v1.0-mini-scene-0061.mcap
```

> GTSAM 4.3a0 (C++) is compiled into the Docker image and installed to `/usr/local`.

---

## Architecture

### Data Pipeline

```
MCAP file
  │
  ├─ /imu  ──────────────► IMU preintegration (CombinedImuFactor)
  ├─ /odom ──────────────► Odometry interpolation at keyframes
  ├─ /gps  ──────────────► WGS84→ENU conversion → GPSFactor (robust)
  ├─ /LIDAR_TOP ─────────► GICP scan-matching → LidarExtrinsicsFactor
  └─ /tf   ──────────────► Initial extrinsic calibration T_BL
                                    │
                                    ▼
                         NonlinearFactorGraph
                                    │
                                    ▼
                       Levenberg-Marquardt optimizer
                                    │
                                    ▼
                    Optimized poses, velocities, biases,
                    and LiDAR extrinsic calibration
```

### Type-Safe Design

All geometric quantities are wrapped with compile-time frame tags from `slam-types`:

```cpp
using BodyFromLidar = Transform<Pose3, Body, LidarTop>;   // extrinsic
using LidarDelta    = Transform<Pose3, LidarTop, LidarTop>; // scan-to-scan
using BodyInEnu     = Pose<Pose3, Body, Enu>;              // body in ENU
using EnuPosition   = TaggedPoint<Point3, Enu>;            // GPS in ENU
```

Frame mismatches (e.g. composing a `BodyFromLidar` with an `EnuPosition`) cause **compile errors**, not runtime bugs. Timestamps use `StrongId<TimestampTag, uint64_t>` to prevent mixing with indices.

### Matrix Types

GTSAM operates on Eigen types internally. The key matrices flowing through the optimizer:

| Matrix | Size | Meaning |
|--------|------|---------|
| `Pose3` | SE(3) | 4×4 rigid transform (stored as Rot3 + Point3) |
| `Vector6` | ℝ⁶ | Tangent-space perturbation (3 rotation + 3 translation) |
| `Matrix6` | 6×6 | Jacobian mapping between tangent spaces |
| Covariance | 6×6 / 3×3 | Marginal uncertainty per state variable |

---

## GPS Outliers and Robust Factors

Real GNSS data contains outliers — multipath reflections, signal occlusion, or sensor glitches can produce position jumps of tens of meters. A standard Gaussian noise model would heavily distort the trajectory to fit these bad measurements.

### The Problem

With a Gaussian (L2) cost, the penalty grows quadratically with residual magnitude. A single 100 m outlier dominates the entire optimization, dragging nearby poses off their true positions.

### The Solution: Huber M-Estimator

We wrap the GPS noise model in a **Huber robust kernel**:

```cpp
auto base  = noiseModel::Isotropic::Sigma(3, 1.0);        // 1 m isotropic
auto huber = noiseModel::mEstimator::Huber::Create(1.345); // threshold
auto gps_noise = noiseModel::Robust::Create(huber, base);
```

The Huber loss behaves quadratically for small residuals (≤ k) and linearly beyond — effectively capping the influence of large outliers without discarding them entirely. The threshold k = 1.345 preserves 95% statistical efficiency under Gaussian noise while providing strong outlier rejection.

In our test scenario we inject 20% corrupted GPS fixes (±100 m random spikes). The optimizer converges cleanly because outlier factors contribute bounded gradient, while the remaining 80% of fixes plus IMU/LiDAR constraints pull the solution toward truth.

---

## LiDAR Extrinsic Calibration

### Motivation

The LiDAR sensor is rigidly mounted on the vehicle body with an unknown (or imprecisely known) transform $T_{BL} \in SE(3)$. Front-end scan-matching (GICP) produces relative motion in the **LiDAR frame**. To fuse these with IMU/GPS data expressed in the **body frame**, we must know — or estimate — $T_{BL}$.

Instead of treating calibration as a separate offline step, we add $T_{BL}$ as a **state variable** in the factor graph and let the optimizer recover it jointly with the trajectory.

### Graph Topology

Each LiDAR odometry measurement creates a **ternary factor** connecting two body poses and the single extrinsic variable:

```
    (X_0)        (X_1)        (X_2)        ...        (T_BL)
      │            │  \          │                    / / /
      └──[F_01]───┘    └──[F_12]───────────────────/  / /
                              └──[F_23]──────────────/  /
                                    ...               /
```

- **One variable** `T_BL` (symbol `L0`) — the extrinsic calibration to estimate.
- **Many ternary factors** `F_ij` — each links poses $X_i$, $X_j$, and $T_{BL}$.

Because `T_BL` participates in every LiDAR factor, the optimizer receives gradient from the entire trajectory, effectively averaging out noise to converge on a globally consistent calibration.

### Measurement Model

Given body poses $X_i, X_j$ and extrinsic $T_{BL}$, the predicted relative LiDAR motion is:

$$\hat{Z}_{ij} = (X_i \cdot T_{BL})^{-1} \cdot (X_j \cdot T_{BL}) = T_{BL}^{-1} \cdot X_i^{-1} \cdot X_j \cdot T_{BL}$$

The residual maps the discrepancy between predicted and measured into the tangent space:

$$e = \text{Log}(Z_{ij}^{-1} \cdot \hat{Z}_{ij}) \in \mathbb{R}^6$$

### Jacobian Derivation via Chain Rule

Rather than expanding Lie algebra by hand, GTSAM's geometry functions return partial Jacobians as **6×6 matrices** when optional references are passed. We chain them using the multivariate chain rule.

Define intermediate LiDAR poses:
$$P_i = X_i \cdot T_{BL}, \quad P_j = X_j \cdot T_{BL}$$

The prediction is:
$$\hat{Z}_{ij} = \text{between}(P_i, P_j)$$

Each step has a known Jacobian:

| Step | Output | Jacobian w.r.t. input |
|------|--------|-----------------------|
| `compose(X_i, T_BL)` → $P_i$ | $\frac{\partial P_i}{\partial X_i}$, $\frac{\partial P_i}{\partial T_{BL}}$ | `H_Pi_Xi`, `H_Pi_TBL` |
| `compose(X_j, T_BL)` → $P_j$ | $\frac{\partial P_j}{\partial X_j}$, $\frac{\partial P_j}{\partial T_{BL}}$ | `H_Pj_Xj`, `H_Pj_TBL` |
| `between(P_i, P_j)` → $\hat{Z}$ | $\frac{\partial \hat{Z}}{\partial P_i}$, $\frac{\partial \hat{Z}}{\partial P_j}$ | `H_pred_Pi`, `H_pred_Pj` |
| `localCoordinates(Z, Ẑ)` → $e$ | $\frac{\partial e}{\partial \hat{Z}}$ | `H_err_pred` |

Final Jacobians by chaining:

$$\frac{\partial e}{\partial X_i} = H_{\text{err\_pred}} \cdot H_{\text{pred\_Pi}} \cdot H_{\text{Pi\_Xi}}$$

$$\frac{\partial e}{\partial X_j} = H_{\text{err\_pred}} \cdot H_{\text{pred\_Pj}} \cdot H_{\text{Pj\_Xj}}$$

$$\frac{\partial e}{\partial T_{BL}} = H_{\text{err\_pred}} \cdot \left( H_{\text{pred\_Pi}} \cdot H_{\text{Pi\_TBL}} + H_{\text{pred\_Pj}} \cdot H_{\text{Pj\_TBL}} \right)$$

The $T_{BL}$ Jacobian is a **sum** because both $P_i$ and $P_j$ depend on $T_{BL}$ (product rule on SE(3)).

### Perturbation Model on SE(3)

To understand *what* those 6×6 matrices encode, consider how derivatives work on Lie groups.

A small perturbation $\delta\xi \in \mathbb{R}^6$ acts on a pose via the exponential map:

$$P' = P \cdot \exp(\delta\xi^\wedge)$$

For the `between` operation $\hat{Z} = P_i^{-1} \cdot P_j$, perturbing both inputs:

$$\hat{Z}' = \exp(-\delta p_i^\wedge) \cdot P_i^{-1} \cdot P_j \cdot \exp(\delta p_j^\wedge)$$

To isolate the perturbation on the right (local coordinates), we slide $\exp(-\delta p_i^\wedge)$ past $\hat{Z}$ using the **Adjoint**:

$$\exp(\xi^\wedge) \cdot T = T \cdot \exp\left((\text{Ad}_{T^{-1}} \xi)^\wedge\right)$$

This gives:

$$\hat{Z}' = \hat{Z} \cdot \exp\left((-\text{Ad}_{\hat{Z}^{-1}} \delta p_i)^\wedge\right) \cdot \exp(\delta p_j^\wedge)$$

Applying the **first-order BCH approximation** (valid because perturbations are infinitesimal, making the Lie bracket $[\delta p_i, \delta p_j]$ second-order negligible):

$$\exp(A^\wedge)\exp(B^\wedge) \approx \exp((A+B)^\wedge) \quad \text{when } \|A\|, \|B\| \ll 1$$

We get:

$$\delta z = -\text{Ad}_{\hat{Z}^{-1}} \, \delta p_i + I_{6 \times 6} \, \delta p_j$$

Reading off the Jacobians:

$$\frac{\partial \hat{Z}}{\partial P_i} = -\text{Ad}_{\hat{Z}^{-1}}, \quad \frac{\partial \hat{Z}}{\partial P_j} = I_{6\times 6}$$

These are exactly the matrices GTSAM returns via `OptionalMatrixType` in `Pose3::between()`.

### Implementation

The factor lives in `src/factor/lidar_extrinsics.hpp`:

```cpp
class LidarExtrinsicsFactor
    : public NoiseModelFactorN<Pose3, Pose3, Pose3> {
    // Keys: X_i, X_j, T_BL
    // measured_Z_: relative LiDAR odometry from GICP

    Vector evaluateError(const Pose3& X_i, const Pose3& X_j,
                         const Pose3& T_BL, ...) const override {
        Pose3 P_i = X_i.compose(T_BL, &H_Pi_Xi, &H_Pi_TBL);
        Pose3 P_j = X_j.compose(T_BL, &H_Pj_Xj, &H_Pj_TBL);
        Pose3 pred = P_i.between(P_j, &H_pred_Pi, &H_pred_Pj);
        Vector6 err = measured_Z_.localCoordinates(pred, nullptr, &H_err_pred);

        // Chain rule assembly
        *H_Xi   = H_err_pred * H_pred_Pi * H_Pi_Xi;
        *H_Xj   = H_err_pred * H_pred_Pj * H_Pj_Xj;
        *H_T_BL = H_err_pred * (H_pred_Pi*H_Pi_TBL + H_pred_Pj*H_Pj_TBL);
        return err;
    }
};
```

### Observability Warning

$T_{BL}$ is only observable when the vehicle experiences **rotational excitation** across multiple axes. Straight-line driving leaves rotation components of the extrinsic unobservable, causing covariance blow-up. Ensure the dataset contains turns, pitch, and roll variations.

---

## Factor Catalog

### 1. LiDAR Extrinsics Factor (Ternary)

Jointly estimates the body-to-LiDAR extrinsic $T_{BL} \in SE(3)$ alongside the trajectory.

$$e = \text{Log}\!\left(Z_{ij}^{-1} \cdot T_{BL}^{-1} \cdot X_i^{-1} \cdot X_j \cdot T_{BL}\right) \in \mathbb{R}^6$$

Keys: `X_i`, `X_j`, `T_BL`. Noise: anisotropic 6-DoF with Huber robust kernel.

### 2. Spatiotemporal Radar Factor (Ternary)

Constrains body velocity via radar Doppler measurements with lever-arm compensation.

$$e = v_r^{\text{meas}} - \left(-\mathbf{u}_R^\top \cdot R_{BR}^\top \cdot \left(R_{BW}^\top \, \mathbf{v}_W + \boldsymbol{\omega}_B \times \mathbf{t}_{BR}\right)\right)$$

Keys: `X_i` (pose), `V_i` (velocity), `T_BR` (radar extrinsic). Noise: 1-DoF scalar with Huber.

### 3. Forward Velocity Scale Factor (Ternary)

Constrains forward-axis displacement between poses to match scaled wheel odometry.

$$e = \frac{(X_i^{-1} \cdot X_j)_x}{\Delta t} - \left((1 + \delta_s) \cdot v_{\text{odom}} - (\boldsymbol{\omega}_B \times \mathbf{l})_x\right)$$

Keys: `X_i`, `X_j`, `S` (scale delta). The scale variable $\delta_s$ is initialized at $-0.03$ (i.e. scale = 0.97) with a tight prior ($\sigma = 0.001$) to ensure slow convergence and prevent divergence on short sequences.

### 4. Non-Holonomic Constraint (Binary)

Enforces zero lateral/vertical velocity at the rear axle in body frame.

$$e = \begin{pmatrix} v_{\text{axle},y} \\ v_{\text{axle},z} \end{pmatrix}, \quad \mathbf{v}_{\text{axle}} = R_{WB}^\top \, \mathbf{v}_W + \boldsymbol{\omega}_B \times \mathbf{t}_{I \to A}$$

Keys: `X_i` (pose), `V_i` (velocity). Noise: 2-DoF diagonal.

### 5. GTSAM Built-in Factors

| Factor | Keys | Dim |
|--------|------|-----|
| `CombinedImuFactor` | $X_i, V_i, B_i, X_j, V_j, B_j$ | 15 |
| `GPSFactor` (robust Huber) | $X_i$ | 3 |
| Pose/Velocity/Bias priors | single variable | 6/3/6 |

---

## Foxglove Visualization

Pass `--output <path>` to produce a **merged** MCAP file containing the original recording plus optimization results — open a single file in Foxglove to see everything:

```bash
./build/nufuse ../../data/nuscenes/mcap/NuScenes-v1.0-mini-scene-0061.mcap \
  --output /tmp/scene-0061-merged.mcap
```

The output copies all source messages (cameras, LiDAR, radar, TF, etc.) and appends the nufuse result channels. Open the single merged file in [Foxglove Studio](https://foxglove.dev/) or [Lichtblick](https://github.com/Lichtblick-Suite/lichtblick).

### Output Topics

| Topic | Schema | TF frames | Content |
|-------|--------|-----------|---------|
| `/nufuse/pose` | `foxglove.FrameTransform` | `map → base_link` | Optimized body trajectory |
| `/nufuse/gps/corrupted` | `foxglove.LocationFix` | — | GPS fixes with injected outlier spikes |
| `/nufuse/gps/clean` | `foxglove.LocationFix` | — | GPS fixes that passed without corruption |
| `/nufuse/extrinsics/estimated` | `foxglove.FrameTransform` | `base_link → LIDAR_TOP_estimated` | Optimized extrinsic |
| `/nufuse/extrinsics/ground_truth` | `foxglove.FrameTransform` | `base_link → LIDAR_TOP_gt` | Ground truth extrinsic |

Nufuse publishes into the **same** transform tree as the source (`map → base_link → sensors`) so all frames resolve in one hierarchy.

### Recommended Panels

- **Map panel** (for GPS): Add `/nufuse/gps/corrupted` and `/nufuse/gps/clean` topics. Set layer colors (e.g. red/green) to distinguish outliers from clean fixes. `LocationFix` messages are **only** visible in the Map panel — they won't appear in 3D.
- **3D panel**: Add `/nufuse/pose` and both `/nufuse/extrinsics/*` topics alongside the original `/tf`. Play back to see the vehicle traverse the trajectory with estimated vs ground truth extrinsic frames.

---

## Project Layout

```
gtsam/
├── CMakeLists.txt
├── Dockerfile
├── pyproject.toml
├── config/
│   └── default.json                # Reference JSON config (all parameters with defaults)
├── scripts/
│   ├── download_data.py
│   └── extract_and_compile_protos.py
├── notebook/
│   └── pose_graph.ipynb
├── tests/
│   ├── test_lidar_extrinsics_factor.cpp    # Analytical Jacobian verification
│   ├── test_spatiotemporal_radar_factor.cpp
│   ├── test_geo_utils.cpp                  # Geodetic utils + nearest-odom lookup
│   └── test_config_loader.cpp              # JSON config loading & defaults
└── src/
    ├── main_nufuse.cpp             # Thin CLI entry point (parses args → pipeline)
    ├── core/
    │   ├── types.hpp               # Frame-tagged transforms, poses, timestamps
    │   ├── frames.hpp              # Coordinate frame tags (Body, Enu, LidarTop, ...)
    │   ├── pipeline_config.hpp     # Full pipeline configuration struct (all noise params)
    │   └── config_loader.hpp/cpp   # Load PipelineConfig from JSON file
    ├── domain/
    │   ├── measurements.hpp        # ImuMeasurement, OdomMeasurement, GnssFix
    │   ├── calibration.hpp         # ExtrinsicCalibration with sensor registry
    │   └── scene_data.hpp          # SceneData container for all sensor streams
    ├── factor/
    │   ├── lidar_extrinsics.hpp/.inl        # Ternary factor (X_i, X_j, T_BL)
    │   └── spatiotemporal_radar.hpp/.inl    # Radar Doppler factor with lever arm
    ├── graph/
    │   ├── builder.hpp/cpp         # Factor graph assembly (FactorGraphBundle)
    │   ├── factor_types.hpp        # StoredImuFactor, StoredGpsFactor, etc.
    │   └── factor_storage.hpp      # Intermediate storage between processing and graph
    ├── io/
    │   ├── mcap_loader.hpp/cpp     # MCAP reader: protobuf + JSON decoding
    │   └── mcap_writer.hpp/cpp     # MCAP writer: results → Foxglove visualization
    ├── pipeline/
    │   └── pipeline.hpp/cpp        # High-level orchestrator: scene + config → results
    ├── processing/
    │   ├── measurement_processor.hpp/cpp  # Chronological keyframe creation
    │   ├── measurement_merger.hpp/cpp     # Chronological sensor interleaving
    │   ├── bias_estimator.hpp/cpp         # Initial gyro bias from IMU vs odom
    │   ├── gps_corruptor.hpp/cpp          # Standalone GPS spike injection (testing)
    │   ├── nhc_generator.hpp/cpp          # Non-holonomic constraint factor generation
    │   ├── forward_velocity_generator.hpp/cpp  # Forward velocity scale factors
    │   ├── lidar_odometry.hpp/cpp         # GICP scan-matching (small-gicp)
    │   ├── radar_processor.hpp/cpp        # RANSAC ego-velocity + factor creation
    │   └── geo_utils.hpp/cpp              # WGS84→ENU, odom interpolation/lookup
    └── results/
        ├── optimizer.hpp/cpp       # LM optimization + covariance extraction
        └── reporter.hpp/cpp        # Console reporting of trajectory/bias/errors
```

### Configuration System

All noise parameters and algorithm settings are defined in `core/pipeline_config.hpp` with compiled defaults. Override any subset via a JSON config file:

```bash
./build/nufuse <mcap_file> --config config/default.json
```

Fields not present in the JSON retain their compiled defaults. See `config/default.json` for the complete schema. Example partial override:

```json
{
  "sensors": { "enable_radar": false },
  "gps": { "sigma": 2.0, "huber_threshold": 2.0 },
  "imu": { "gravity": 9.81 }
}
```

## Dependencies

### Via FetchContent

- [foxglove/mcap](https://github.com/foxglove/mcap) v2.1.3 — MCAP C++ reader
- [nlohmann/json](https://github.com/nlohmann/json) v3.11.3 — JSON parsing
- [small-gicp](https://github.com/koide3/small_gicp) — fast GICP registration

### System (installed in Docker image)

- GTSAM 4.3a0, protobuf, liblz4, libzstd

### Python

```bash
uv sync  # installs gtsam wheel, numpy, matplotlib, ipykernel
```

---

## Tests

Four test binaries verify analytical Jacobians, utilities, and configuration:

```bash
./build/test_lidar_extrinsics_factor      # 9 tests — Jacobian correctness
./build/test_spatiotemporal_radar_factor   # 7 tests — Jacobian correctness
./build/test_geo_utils                    # 7 tests — LLA→ENU, nearest odom lookup
./build/test_config_loader                # 7 tests — JSON loading, defaults, errors
```

Factor tests use `gtsam::internal::testFactorJacobians()` with step size $\delta = 10^{-5}$ and tolerance $10^{-6}$. Config tests verify partial override semantics (unspecified fields keep compiled defaults).

---

## Results — End-Position Error (m)

Evaluated on NuScenes-mini (~20 s scenes). GPS limited to first 3 fixes (initialization anchor only); 20% of fixes corrupted with ±100 m spikes + Huber robust kernel. Config 1 uses ground-truth position for the first keyframe (pure dead-reckoning test).

**Key improvement:** gyro bias prior is initialized from IMU vs odometry comparison and adapted per observability regime (tight in dead-reckoning, loose with LiDAR). This prevents spurious heading drift in pure IMU+NHC mode while allowing full bias calibration when exteroceptive heading references are available.

| Scene | C1 | C2 | C3 | C4 | C5 | C6 | C7 |
|-------|----|----|----|----|----|----|-----|
| 0061 | 1.5 | 2.0 | 1.7 | 2.0 | 6.0 | 2.8 | **2.4** |
| 0103 | 4.7 | 4.3 | 6.5 | 4.3 | 4.7 | 6.1 | **6.8** |
| 0553 | 0.01 | 0.55 | 0.53 | 0.55 | 1.3 | 13 | **0.53** |
| 0655 | 3.8 | 4.5 | 15 | 4.4 | 17 | 12 | **18** |
| 0757 | 0.92 | 1.3 | 2.4 | 1.3 | 5.0 | 7.1 | **2.6** |
| 0796 | 10 | 10 | 32 | 9.9 | 90 | 27 | **36** |
| 0916 | 3.3 | 2.9 | 12 | 2.9 | 7.3 | 2.8 | **11** |
| 1077 | 8.9 | — | 80 | — | 100 | — | **84** |
| 1094 | 7.7 | 7.5 | 14 | 7.5 | 25 | 13 | **15** |
| 1100 | 0.24 | 0.39 | 0.31 | 0.38 | 0.57 | 0.89 | **0.28** |

"—" = crash (keyframing requires LiDAR timestamps when LiDAR is disabled).

### Configuration Key

| ID | Sensors | Flags |
|----|---------|-------|
| **C1** | IMU + NHC + FwdVel (GT init) | `--no-gps --no-lidar --no-radar --init-from-gt` |
| **C2** | GPS(3) + IMU + NHC + FwdVel | `--no-lidar --no-radar --gps-max 3` |
| **C3** | GPS(3) + IMU + LiDAR + NHC + FwdVel | `--no-radar --gps-max 3` |
| **C4** | GPS(3) + IMU + Radar + NHC + FwdVel | `--no-lidar --gps-max 3` |
| **C5** | GPS(3) + IMU + LiDAR + NHC | `--no-radar --no-fwdvel --gps-max 3` |
| **C6** | GPS(3) + IMU + Radar + NHC | `--no-lidar --no-fwdvel --gps-max 3` |
| **C7** | Full (all sensors) | `--gps-max 3` |

### Findings

1. **Data-driven bias initialization is essential.** Computing initial gyro bias from IMU vs odometry comparison ($\text{bias} \approx \bar{\omega}_\text{imu} - \bar{\omega}_\text{odom}$ over first 100 samples) and using a tight prior ($\sigma_\text{gyro} = 0.003$ rad/s) reduced C1 errors by 3–6× on turning scenes (1094: 43 → 7.7 m, 0796: 37 → 10 m).
2. **Adaptive priors by observability regime.** Without LiDAR, the gyro prior must be tight (heading is unobservable). With LiDAR providing dense rotation info, the prior is loosened ($\sigma = 0.1$) to allow bias calibration against LiDAR heading.
3. **FwdVel + NHC achieve near-LiDAR accuracy in dead-reckoning.** C1/C2 now match or beat C3 on most scenes — the tight gyro prior + FwdVel scale provide better heading/speed than LiDAR GICP (which has systematic heading errors from planar scenes).
4. **LiDAR GICP heading is unreliable** on flat urban roads (0796: 32 m, 0655: 15 m). The GICP algorithm struggles with rotational alignment on planar/symmetric scenes, requiring the optimizer to absorb heading error into gyro bias.
5. **Radar provides marginal improvement** (C4 ≈ C2) — nuScenes radar is sparse with high noise ($\sigma = 1.5$ m/s).
6. **Dead-reckoning limit** for consumer MEMS IMU: ~4% of distance on turning scenes (heading error from gyro noise random walk + accelerometer model mismatch), sub-meter on straight drives.
