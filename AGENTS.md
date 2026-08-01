# AGENTS.md

Single source of truth for agent context in this repository.
`CLAUDE.md`, `GEMINI.md`, and `.github/copilot-instructions.md` are symlinks to this file.
Keep the six section headings below — `scripts/setup.py` feeds this document to
`codebase-memory-mcp` as the project ADR.

## PURPOSE

Learning repository for C++ SLAM optimization and ML pipeline work.
Structured for AI agent-assisted development with consistent conventions across
C++, Python, CMake, and Docker. Each sub-project is self-contained: its own
build, tests, and dependencies.

Because this is a learning repo, clarity beats cleverness. Code that a reader
can follow is worth more than code that is maximally short or general.

## STACK

- **C++20** — CMake with FetchContent, GoogleTest (`dummy_project` is C++17)
- **Python** — pinned per project; `uv` for environments, Ruff for lint/format, pytest for tests
- **Pybind11** — C++/Python bindings, NumPy via `pybind11/numpy.h`
- **Docker** — one image per containerized sub-project, non-root `devuser`, `uv sync --frozen`
- **codebase-memory-mcp** — v0.9.0, structural knowledge graph over the repo

## ARCHITECTURE

Monorepo. Sub-projects differ in shape — do not assume every project has a
build system or a container.

| Project | CMake | Docker | Dev Container | Python |
|---|---|---|---|---|
| `projects/gtsam` | C++20 | yes | yes | 3.13 |
| `projects/slam-types` | C++20 | yes | yes | >=3.11 |
| `projects/symforce` | C++20 | yes | yes | 3.12 |
| `projects/vision-sandbox` | — | — | — | >=3.11 |
| `projects/visual-inference` | — | — | — | >=3.11 |
| `dummy_project` | C++17 | yes | yes | >=3.11 |

`dummy_project/` is the template for new sub-projects.

Shared, repo-root:

- `.devcontainer/<project>/devcontainer.json` — all dev container definitions live here, **not** inside sub-projects. Each mounts the monorepo root at `/workspace`.
- `scripts/setup.sh` + `scripts/setup.py` — index the repo and seed this file as the ADR
- `scripts/download_data.py` — dataset fetch into `data/` (gitignored)
- `.agents/skills/<name>/SKILL.md` — cross-agent skills
- `.vscode/mcp.json` — MCP server definition, repo-scoped and committed
- `AGENTS.md` — this file

Not in version control: `data/`, `build/`, `.venv/`, `.codebase-memory/`.

## PATTERNS

- **CMake**: `target_*` commands only, no directory-scoped globals. FetchContent for dependencies. Build with `cmake -B build && cmake --build build`.
- **C++**: Google Style. Doxygen `///` with `@param`/`@return` on every class and public method.
- **Python**: type hints on all function signatures. `uv` for envs, `ruff` for lint and format, `pytest` for tests.
- **Testing**: GoogleTest for C++, pytest for Python. Tests live next to the code they cover. Prefer real objects over mocks; mock only at process boundaries.
- **Docker**: one image per containerized project. Non-root `devuser`. `uv sync --frozen` so lockfiles are authoritative.
- **Pybind11**: binding translation units under `src/bindings/` (see `projects/gtsam/src/bindings/`). Accept and return NumPy arrays without copying where the buffer protocol allows it.
- **Pinning**: third-party binaries are installed from a pinned release and checksum-verified, never from an unpinned `curl | bash`.

Apply a rule only where it makes sense — `vision-sandbox` and `visual-inference`
are Python-only, so CMake, Docker, and Pybind11 rules do not apply to them.

## TRADEOFFS

- **codebase-memory-mcp over grep-only workflows** — gives agents a structural graph (callers, imports, blast radius) that text search cannot answer. It supplements search rather than replacing it.
- **Single `AGENTS.md` over a directory of ADR files** — every agent reads a fixed filename, so one file plus symlinks reaches all of them with no build step. The previous `.agent_knowledge/` tree required `setup.py` to concatenate fragments and stripped metadata that nothing consumed. Split into per-file ADRs retrieved through `manage_adr` only once this file outgrows roughly 10–15 KB.
- **Root-level `.devcontainer/`** — containers mount the monorepo root so `scripts/` and this file are always reachable, at the cost of the definitions not sitting next to the projects they serve.
- **Dev tooling baked into images, not `postCreateCommand`** — slower image builds, but container startup is fast and works offline.

## PHILOSOPHY

**Think before coding.**

- State assumptions explicitly. If something is uncertain, ask and surface the tradeoffs.
- If multiple interpretations exist, outline them — do not silently pick one.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, pause and name what is confusing.

**Prioritize simplicity.**

- Write the minimum code that solves the problem.
- No flexibility or configurability that was not requested.
- If you wrote 200 lines and it could be 50, rewrite it.
- Ask: would a senior engineer call this overcomplicated? If yes, simplify.

**Use the knowledge graph where it is strong.**

`codebase-memory-mcp` is available and indexes this repo. Route by question type:

- *Structural* questions — who calls this, what imports that, what breaks if I change this signature — use the graph (`search_graph`, `trace_path`, `query_graph`, `get_code_snippet`).
- *Textual* questions — find this literal, this config key, this error string — use ordinary search and file reads. Grep is the right tool here.
- Before non-trivial structural work in an unfamiliar area, `get_architecture` is a cheap orientation step. Recommended, not required.
- A clean graph result is not proof of absence. Dynamic dispatch, string-built names, and unindexed files are invisible to it. Confirm with a text search before concluding something is unused.

**Changing decisions.**

Edit this file in the same change that alters the behaviour it describes, then
run `python3 scripts/setup.py` to re-seed. Do not quietly diverge from what is
written here — if a decision no longer holds, update it or raise it.
