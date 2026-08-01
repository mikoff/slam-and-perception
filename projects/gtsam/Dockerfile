FROM debian:trixie-slim

RUN apt-get update && apt-get install -y \
    build-essential \
    clangd \
    clang-format \
    clang-tidy \
    cmake \
    curl \
    gdb \
    git \
    protobuf-compiler \
    libprotobuf-dev \
    liblz4-dev \
    libzstd-dev \
    python3 \
    python3-dev \
    python3-pip \
    docker.io \
    && rm -rf /var/lib/apt/lists/*

COPY --from=ghcr.io/astral-sh/uv:latest /uv /bin/uv

# Build-time Python deps for GTSAM wrapper generation + MCAP proto extraction
RUN pip3 install --break-system-packages pyparsing numpy setuptools wheel mcap protobuf

# Build and install GTSAM develop (C++ + Python bindings)
RUN git clone --depth=1 --branch develop https://github.com/borglab/gtsam.git /tmp/gtsam \
    && cmake -S /tmp/gtsam -B /tmp/gtsam-build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-Wno-error=free-nonheap-object -Wno-error=array-bounds" \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_UNSTABLE=OFF \
    -DGTSAM_BUILD_PYTHON=ON \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=OFF \
    -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF \
    -DGTSAM_USE_BOOST_FEATURES=OFF \
    -DGTSAM_PYTHON_VERSION=3 \
    && cmake --build /tmp/gtsam-build -j$(nproc) \
    && cmake --install /tmp/gtsam-build \
    && mkdir -p /opt/wheels \
    && pip3 wheel --no-deps -w /opt/wheels /tmp/gtsam-build/python \
    && rm -rf /tmp/gtsam /tmp/gtsam-build \
    && ldconfig

# Install codebase-memory-mcp (dev tooling — belongs in image, not postCreateCommand)
# Pinned release tarball; the upstream install subcommand ignores --skip-config, so it is never run.
ARG CBM_VERSION=v0.9.0
ARG TARGETARCH=amd64
RUN set -eux; \
    base="https://github.com/DeusData/codebase-memory-mcp/releases/download/${CBM_VERSION}"; \
    tarball="codebase-memory-mcp-ui-linux-${TARGETARCH}.tar.gz"; \
    cd /tmp; \
    curl -fsSL -O "${base}/${tarball}"; \
    curl -fsSL -O "${base}/checksums.txt"; \
    grep " ${tarball}\$" checksums.txt | sha256sum -c -; \
    tar -xzf "${tarball}" codebase-memory-mcp; \
    install -m 755 codebase-memory-mcp /usr/local/bin/codebase-memory-mcp; \
    rm -f "${tarball}" checksums.txt codebase-memory-mcp

RUN useradd -m -s /bin/bash devuser
WORKDIR /workspace
