FROM debian:trixie-slim

# Install C++ build system and tools
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    gdb \
    git \
    python3 \
    libgmp-dev \
    && rm -rf /var/lib/apt/lists/*

# Install uv for Python management
COPY --from=ghcr.io/astral-sh/uv:latest /uv /bin/uv

# Install codebase-memory-mcp (dev tooling — belongs in image, not postCreateCommand)
RUN curl -fsSL https://raw.githubusercontent.com/DeusData/codebase-memory-mcp/main/install.sh \
    | bash -s -- --ui --skip-config --dir=/usr/local/bin

RUN useradd -m -s /bin/bash devuser
# Set default working directory
WORKDIR /workspace
