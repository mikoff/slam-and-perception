FROM debian:bookworm-slim

# Install C++ build system and tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    gdb \
    git \
    python3 \
    && rm -rf /var/lib/apt/lists/*

# Install uv for Python management
COPY --from=ghcr.io/astral-sh/uv:latest /uv /bin/uv

RUN useradd -m -s /bin/bash devuser
# Set default working directory
WORKDIR /workspace
