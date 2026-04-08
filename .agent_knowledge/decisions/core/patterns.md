---
topic: Coding Patterns
tags: [cmake, c++, python, docker, testing, pybind11]
related_nodes: [stack.md, philosophy.md]
---
## PATTERNS

- **CMake**: `target_*` only, no global variables. FetchContent for deps. `cmake -B build && cmake --build build`.
- **C++**: C++20, Google Style, Doxygen `///` with `@param`/`@return` on every class and method.
- **Python**: Type hints on all functions. `uv` for envs. `ruff` for lint/format. `pytest` for tests.
- **Testing**: GTest for C++, pytest for Python. Tests next to source. No brittle mocks.
- **Docker**: One image per sub-project. Non-root user. `uv sync --frozen`.
- **Pybind11**: `bindings/` module. NumPy via `pybind11/numpy.h`, no unnecessary copies.
- **Markdown**: YAML frontmatter required (`topic`, `tags`, `related_nodes`).
