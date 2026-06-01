# 🛰️ SLAM / Perception Engineer Projects

## Idea
A list of weekly projects which expose core ideas and concept in SLAM/Perception.
Every small project is aimed at demonstration of one specific tool/framework/algorithm and is supplemented by a short note taken while reading the associated paper.

## Principles
- 🐳 `Docker`-first for all projects for reproducibility.
- ⚡ `uv` for all Python.
- 💡 Pre-work evenings: 30-60 minutes on weekday. Pull images, data, start builds.
- 🛠️ Weekend = 8 hours: 2 hrs of study + 5–6 hours hands-on. Stuck > 90 min → switch the topic.
- 🧠 GPU target: should run on RTX 3060 with 12GB of RAM (or Colab Pro).

## 🚀 Usage
1. 📁 Copy `dummy_project` into the relevant directory.
2. 📚 Read paper/documentation and start prototyping.
3. 🧪 Write the code and demonstrate results.
4. ✅ Wrap up the project and switch to the next one.

## Projects

- **[slam-types](projects/slam-types/)** - A C++20 header-only library of compile-time safety primitives for SLAM codebases. Provides `StrongId` for type-safe identifiers (preventing ID mix-ups between poses, landmarks, etc.) and frame-tagged transforms that enforce coordinate-frame consistency at compile time, catching frame mismatches as build errors with zero runtime overhead.

- **[NuFuse / gtsam](projects/gtsam/)** - A tightly-integrated SLAM back-end built on GTSAM that fuses IMU preintegration, GNSS fixes, and LiDAR scan-matching odometry into a single factor graph. The optimizer jointly estimates the vehicle trajectory and the LiDAR extrinsic calibration, using robust factors to gracefully handle GPS outliers. Evaluated on the nuScenes mini dataset via MCAP.

- **[symforce](projects/symforce/)** - An exploration of symbolic factor graph optimization using SymForce. Covers uncertainty representation and whitening, Lie group notation for poses and odometry, and the derivation of residuals for SE(3) factors - providing the mathematical foundation for automatic Jacobian generation in SLAM back-ends.