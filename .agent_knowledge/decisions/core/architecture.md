---
topic: Repository Architecture
tags: [monorepo, layout, devcontainer]
related_nodes: [purpose.md, stack.md]
---
## ARCHITECTURE

Monorepo with sub-project directories. Each has its own
`CMakeLists.txt`, `Dockerfile`, `pyproject.toml`, and `.devcontainer/`.

- `dummy_project/` — template for new sub-projects

Shared:
- `scripts/setup.sh` + `setup.py` — binary install, indexing, ADR seeding
- `.agent_knowledge/decisions/` — ADR source (core/ and generated/)
- `.codebase-memory/adr.md` — compiled ADR blob (derived)
