## ARCHITECTURE

Monorepo with sub-project directories. Each has its own
`CMakeLists.txt`, `Dockerfile`, `pyproject.toml`, and `.devcontainer/`.

- `dummy_project/` — template for new sub-projects

Shared:
- `scripts/setup.sh` + `setup.py` — binary install, indexing, ADR seeding
- `.agent_knowledge/decisions/` — ADR source (core/ and generated/)
- `.codebase-memory/adr.md` — compiled ADR blob (derived)

## PATTERNS

- **CMake**: `target_*` only, no global variables. FetchContent for deps. `cmake -B build && cmake --build build`.
- **C++**: C++20, Google Style, Doxygen `///` with `@param`/`@return` on every class and method.
- **Python**: Type hints on all functions. `uv` for envs. `ruff` for lint/format. `pytest` for tests.
- **Testing**: GTest for C++, pytest for Python. Tests next to source. No brittle mocks.
- **Docker**: One image per sub-project. Non-root user. `uv sync --frozen`.
- **Pybind11**: `bindings/` module. NumPy via `pybind11/numpy.h`, no unnecessary copies.
- **Markdown**: YAML frontmatter required (`topic`, `tags`, `related_nodes`).

## PHILOSOPHY

- Call `get_architecture` and `manage_adr(mode="list")` before any non-trivial task.
- Do not deviate from decisions without discussion.
- New decisions go in `.agent_knowledge/decisions/generated/`, then run `setup.py`.
- Follow all patterns in PATTERNS — they are mandatory, not suggestions.
- Think before coding:
  - State your assumptions explicitly, if something uncertain, ask and surface tradeoffs.
  - If multiple interpretations exist, outline them - don't choose silently.
  - If a simpler approach exists, say so. Push back when warranted.
  - If something is unclear, pause and name what is confusing.
- Prioritize simplicity:
  - Write minimum code that solves the problem.
  - No "flexibility" or "configurability" that wasn't requested.
  - If you write 200 lines and it could be 50, rewrite it.
  - Ask yourself: "would a senior engineer say this is overcomplicated?" If yes, simplify.

## PURPOSE

Learning repository for C++ SLAM optimization and ML pipeline work.
Structured for AI agent-assisted development with strict consistency
across C++, Python, CMake, and Docker. Each sub-directory is a
self-contained project with its own build, test, and container setup.

## STACK

- **C++20** — CMake with FetchContent, GoogleTest
- **Python 3.12** — uv, Ruff, pytest
- **Pybind11** — C++/Python bindings, numpy via `pybind11/numpy.h`
- **Docker** — one image per sub-project, non-root user, `uv sync --frozen`
- **codebase-memory-mcp** — knowledge graph, ADR management, auto-sync indexing

## TRADEOFFS

- **codebase-memory-mcp** — structural graph indexing.
- **ADR blob compiled from files** — binary requires one 6-section document; `setup.py` concatenates individual files from `decisions/`.
- **Dev Container mounts monorepo root** — ensures `scripts/` and `.agent_knowledge/` are always accessible.