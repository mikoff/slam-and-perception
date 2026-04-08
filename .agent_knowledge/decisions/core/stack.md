---
topic: Technology Stack
tags: [c++20, python, cmake, docker, pybind11]
related_nodes: [patterns.md, architecture.md]
---
## STACK

- **C++20** — CMake with FetchContent, GoogleTest
- **Python 3.12** — uv, Ruff, pytest
- **Pybind11** — C++/Python bindings, numpy via `pybind11/numpy.h`
- **Docker** — one image per sub-project, non-root user, `uv sync --frozen`
- **codebase-memory-mcp** — knowledge graph, ADR management, auto-sync indexing
