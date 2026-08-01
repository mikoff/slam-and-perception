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
    python3 \
    libgmp-dev \
    && rm -rf /var/lib/apt/lists/*

# Build and install GTSAM 4.3a0 (C++ only, no Boost) for integration tests
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
