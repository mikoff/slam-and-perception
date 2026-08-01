FROM debian:trixie-slim

# Install C++ build system and tools
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

# Install uv for Python management
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
# Set default working directory
WORKDIR /workspace
