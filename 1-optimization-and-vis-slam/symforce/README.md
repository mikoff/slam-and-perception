---
topic: Symforce optimization
tags: [symforce, slam, factor graph, optimization, coordinate transforms]
scope: Local to symforce optimization
---

# Theory
## Uncertainty Representation (Normalization)
Uncertainty is typically represented by a Covariance Matrix $\mathbf{\Sigma}$. Its inverse is the Information matrix $\mathbf{\Omega} = \mathbf{\Sigma}^{-1}$.
The typical operation performed before creating a factor in the graph is to normalize, or "whiten" the error to treat all variables uniformly.
This can be done:
- for diagonal noise model by pre-multiplying each scalar of the error vector by its inverse standard deviation;
- for complex error models by pre-multiplying the error vector by the square root information matrix $\mathbf{W}$ (so $\mathbf{W}^T \mathbf{W} = \mathbf{\Omega})$.

By doing this we guarantee that all the values become unitless and have the same weights.

**It converts physical units (meters) into statistical units (sigmas)**.

## Notations
 $$\mathbf{T}_a = \mathbf{T}_{\text{world} \leftarrow a}$$
  the pose of frame $a$ relative to the world frame. The pose is expressed in the world frame. However, it maps points from hte local frame to the global frame.
  - _Code variable_: `T_wa`, `toWorldFromA`
---

 $$\mathbf{p}_{\text{world}} = \mathbf{T}_a \mathbf{p}_a$$
  - _Code variables_: `p_w = T_wa * p_a`
---

 $$\mathbf{T}_{\text{world} \leftarrow b} = \mathbf{T}_{\text{world} \leftarrow a} \mathbf{T}_{a \leftarrow b}$$
  pose composition.
  - _Code variables_: `T_wb = T_wb * T_ab`
---

 $$\mathbf{T}_{ab} = \mathbf{T}_a^{-1} \mathbf{T}_b$$
  odometry, or relative pose.
  - _Code variables_: `T_ab = T_wa.inverse() * T_wb`
  - _SymForce_: `between(a, b)`
---

 $$\mathbf{e}_{ab} = \log \left( \mathbf{T}_{ab}^{-1} \hat{\mathbf{T}}_{ab} \right)^\vee$$
  error between a measured relative pose and a predicted relative pose, $\vee$ is the vee operator, which turns a Lie algebra matrix into a vector in Euclidean space (since the resulting matrix is skew-symmetric, we take only the essential numbers and pack them into a minimal column vector).
  - _Code variables_: `error_ab = log(T_ab_meas.inverse() * T_ab_pred).vee()`
  - _SymForce_: `local_coordinates(T_ab_meas, T_ab_pred)`, $\ominus$
---

 $$\mathbf{e}_{\text{between}} = \mathbf{W}_{ab}\mathbf{e}_{ab}$$
  the between factor that penalizes the difference between the predicted relative pose and the measured odometry, scaled by certainty (measurement information matrix).
  - _Code variables_: `res_ab = weightMatrix_ab * error_ab`
---

 $$\mathbf{e}_{\text{landmark}} = \mathbf{W_\text{meas}}\left(\mathbf{T}_{a}^{-1} \mathbf{p}_{\text{world}} - \mathbf{p}_{a} \right)$$
  the pose-to-landmark factor. It penalizes the difference between where a landmark should appear in the local frame of $a$ and where it was actually measured ($\mathbf{p}_a$).
  - _Code variables_: `res_lmrk = weightMatrix_meas * (T_wa.inverse() * p_w - p_a_meas)`
---


## Active and passive transformations.
**Note:** many people talk of active and passive transformations.
1. Active Transformation = the reference frame perspective.
  It describes how the reference frame itself moves w.r.t. the world. 
  It is active because the global world is fixed, so the origin is taken and moved to a new physical location to create frame $\mathbf{a}$.
2. Passive transformation = the point perspective.
  It converts sensor data between frames. The physical objects stay still, but the mathematical grid used to measure it is swapped.

**The takeaway:** an active transformation of a frame is equivalent to a passive transformation of the points in the opposite direction. Therefore, the matrix $\mathbf{T}_{\text{world}\leftarrow a}$ performs a **passive** transformation on local points, representing the exact same numerical values as the active physical movement of the frame from the _world_ to $a$ ($\mathbf{T}_{\text{world} \to a}$)

## Error definition
$$\overbrace{\hat{\mathbf{T}}}^{\text{\scriptsize predicted}} = \overbrace{\mathbf{T}}^{\text{\scriptsize anchor}} \underbrace{\exp(\mathbf{e}_{\text{right}}^\wedge)}_{\text{\scriptsize local twist}} \;\xrightarrow{\mathbf{T}^{-1} \cdot}\; \underbrace{\mathbf{T}^{-1} \hat{\mathbf{T}}}_{\text{\scriptsize pose difference}} = \exp(\mathbf{e}_{\text{right}}^\wedge) \;\xrightarrow{\log}\; \log(\mathbf{T}^{-1} \hat{\mathbf{T}}) = \overbrace{\mathbf{e}_{\text{right}}^\wedge}^{\text{\scriptsize Lie algebra}} \;\xrightarrow{\vee}\; \underbrace{\mathbf{e}_{\text{right}}}_{\text{\scriptsize error vector}} = \log(\mathbf{T}^{-1} \hat{\mathbf{T}})^\vee$$

* **The Anchor:** The raw odometry measurement ($\mathbf{T}$) is a fixed, immovable reference point.
* **The Error:** $\mathbf{e}_{\text{right}} = \log(\mathbf{T}^{-1} \hat{\mathbf{T}})^\vee$ is the exact geometric vector pointing from this anchor to a dynamically predicted pose ($\hat{\mathbf{T}}$).
* **The Pull:** Optimization adjusts the global pose estimates to shrink this error vector to zero, effectively pulling the prediction until it perfectly aligns with the anchor ($\mathbf{T} \exp(\mathbf{e}_{\text{right}}^\wedge) \to \mathbf{T}$).

## The Universal Subtraction Operator $\ominus$
In standard flat spaces (like 2D pixel coordinates), the error between two points can be found using standard subtraction: $\mathbf{e} = \mathbf{b} - \mathbf{a}$.

Because transformation matrices live on a curved manifold, standard subtraction ($\mathbf{b} - \mathbf{a}$) breaks the geometry. To fix this, the generalized subtraction operator was created, usually written as $\ominus$.

$$\mathbf{b} \ominus \mathbf{a} \triangleq \log(\mathbf{a}^{-1} \mathbf{b})^\vee$$

`local_coordinates(a, b)` is the exact programmatic implementation of the $\ominus$ operator.

## Whitening
Factors are directly scaled inside the residual function by multiplying the raw error vector by the square-root information matrix $\mathbf{W}$.
$$\mathbf{e}_{\text{whitened}} = \mathbf{W} \mathbf{e}_{\text{raw}}$$

This ensures the optimizer minimizes the Mahalanobis distance:
$$\|\mathbf{W} \mathbf{e}_{\text{raw}}\|^2_2 = \mathbf{e}_{\text{raw}}^T \Omega \mathbf{e}_{\text{raw}}$$

## Robustification
To handle outliers, standard least squares minimizes $\frac{1}{2} \|e_{whitened}\|^2$. Robustifiers wrap this squared norm in a loss function $\rho(x)$ that grows sub-quadratically (e.g., Huber or Barron loss) to reduce the pull of massive errors.

$$\text{Cost} = \sum \rho(\| e_{whitened} \|^2)$$

Note: In SymForce, robustification is typically applied after the residual function returns $\mathbf{e}_{\text{whitened}}$, using the optimizer's built-in loss wrappers (like `sym.BarronLoss`), rather than writing it inside the raw matrix equations.


# Pose-Graph SLAM -- SymForce C++ Optimizer

End-to-end pose-graph SLAM using [SymForce](https://github.com/symforce-org/symforce) for automatic factor Jacobian generation and a Levenberg–Marquardt back-end in C++. Supports both 2-D (SE2) and 3-D (SE3) datasets from the iSAM2 benchmark suite.

---

## Architecture

### Factor definitions (Python / SymForce codegen)

Eight factor types are defined symbolically in `notebook/generate_factors.ipynb` and compiled to optimised C++ headers in `inc/gen/`:

| Factor | Geometry | Kernel |
|---|---|---|
| `between_factor_se2/3` | odometry | Gaussian |
| `loop_closure_factor_se2/3` | loop closure | **Barron** (robust) |
| `pose_to_landmark_factor_se2/3` | landmark observation | **Barron** (robust) |
| `prior_factor_se2/3` | absolute prior | Gaussian |

The Barron kernel generalises common M-estimators (Geman–McClure at `α = −2`, Cauchy at `α = 0`, Huber at `α = 1`), controlled by `barron_alpha` and `barron_delta`. It suppresses outlier loop closures and landmark observations.

### C++ class hierarchy

```
pose_graph_optimizer (main)
  └─ CFactorGraph<G>               factor_graph.hpp / .inl / .cpp
       ├─ CFactorGraphStorage<G>   factor_graph_storage.hpp / .inl / .cpp
       │    ├─ sym::Values<double> -- all variables (poses, landmarks, measurements)
       │    └─ sym::Factor<double> -- residual + Jacobian handles (SymForce-generated)
       └─ sym::Optimizer<double>   (SymForce LM sparse solver)

Geometry policies (geometry_types.hpp / .cpp):
  SE2  -- sym::Pose2d, Eigen::Vector2d, 3×3 info matrices
  SE3  -- sym::Pose3d, Eigen::Vector3d, 6×6 info matrices

Dataset parsers (dataset_parser.hpp / .cpp):
  parseDatasetSE2  -- EDGE2 / ODOMETRY / LANDMARK tokens
  parseDatasetSE3  -- EDGE3 / POINT3 tokens
  detectDatasetGeometry -- auto-selects pipeline from first recognised token
```

### Geometry policy pattern

`CFactorGraph<G>` is templated on a **geometry policy** (`SE2` or `SE3`). Each policy provides type aliases (`Pose`, `Position`, `PoseInfoMatrix`, …), static `build*Factor()` methods that store measurement parameters into `sym::Values` and return the corresponding `sym::Factor`, and JSON serialisers. The entire optimisation pipeline is shared across geometries.

### Variable key scheme (`sym::Values`)

| Key | Meaning |
|---|---|
| `('P', id)` | Pose `id` |
| `('L', id)` | Landmark `id` |
| `('m', k)` / `('I', k)` | Between-factor measurement / sqrt-info |
| `('c', k)` / `('C', k)` | Loop-closure measurement / sqrt-info |
| `('o', k)` / `('J', k)` | Landmark observation / sqrt-info |
| `('a/b', k)` / `('d/B', k)` | Barron α / δ for loop-closure / landmark factors |
| `('q', k)` / `('Q', k)` | Prior pose / sqrt-info |
| `('e', 0)` | Global epsilon (numerical stability) |

### Optimisation back-end

`CFactorGraph::optimize()` wraps `sym::Optimizer<double>` with LM parameters (200 iterations, `early_exit_min_reduction = 1e-8`).

---

## Supported datasets

| File | Geometry | Ground truth |
|---|---|---|
| `city10000.txt` | SE2 | ✅ |
| `cityTrees10000.txt` | SE2 + landmarks | ✅ |
| `manhattanOlson3500.txt` | SE2 (`ODOMETRY` token) | ✅ |
| `sphere400.txt` | SE3 | -- |
| `sphere2500.txt` | SE3 | ✅ |
| `torus10000.txt` | SE3 | -- |
| `torus2000Points.txt` | SE3 + landmarks | -- |

---

## Build & run

### 1. Download datasets

Datasets are stored in the top-level `data/` folder of the repo (shared across all projects). Run once:

```bash
python /workspace/1-optimization-and-vis-slam/symforce/scripts/download_data.py
```

Files are downloaded to `/workspace/data/iSAM2/` and skipped if already present.

### 2. Build

```bash
cd /workspace/1-optimization-and-vis-slam/symforce
source .venv/bin/activate
cd build && cmake --build . -j$(nproc) && cd ..
```

### 3. Run

Geometry is auto-detected from the dataset file. JSON results are written to `build/`:

```bash
./build/pose_graph_optimizer ../../data/iSAM2/cityTrees10000.txt ./build
./build/pose_graph_optimizer ../../data/iSAM2/torus10000.txt     ./build
./build/pose_graph_optimizer ../../data/iSAM2/sphere2500.txt     ./build
```

Results are written to `build/<stem>.json` (kept out of version control alongside the build artefacts).

---

## Notebooks

Both notebooks live in `notebook/` and use the **Python (symforceproject)** kernel.

### `generate_factors.ipynb`

Symbolically defines and code-generates all 8 SymForce factor types into `inc/gen/`:

- **Section 1** -- define and codegen all 8 factors.
- **Section 2** -- synthetic 4-pose demo with an outlier landmark and a false loop closure, demonstrating Barron kernel rejection.

### `run_and_visualize_results.ipynb`

Builds the C++ optimizer, runs it on every dataset, and visualises the results:

- **Cell 1 -- Build & Run** -- builds `pose_graph_optimizer`, runs it on all 7 datasets, and prints the error reduction.
- **Cell 2 -- Helpers** -- SE2/SE3 file parsers, dead-reckoning.
- **Cell 3 -- Visualisation** -- one figure per dataset.
- **Cell 4 -- GT error analysis** -- for the 4 datasets with ground truth, optimization summary and statistics.