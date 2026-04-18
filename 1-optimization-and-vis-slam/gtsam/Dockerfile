FROM debian:trixie-slim

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    curl \
    gdb \
    git \
    python3 \
    && rm -rf /var/lib/apt/lists/*

# Build and install GTSAM 4.3a0 (C++ only, no Boost)
RUN git clone --depth=1 --branch 4.3a0 https://github.com/borglab/gtsam.git /tmp/gtsam \
 && cmake -S /tmp/gtsam -B /tmp/gtsam-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_PYTHON=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=OFF \
        -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF \
        -DGTSAM_USE_BOOST_FEATURES=OFF \
 && cmake --build /tmp/gtsam-build -j$(nproc) \
 && cmake --install /tmp/gtsam-build \
 && rm -rf /tmp/gtsam /tmp/gtsam-build \
 && ldconfig

COPY --from=ghcr.io/astral-sh/uv:latest /uv /bin/uv

RUN useradd -m -s /bin/bash devuser
WORKDIR /workspace
