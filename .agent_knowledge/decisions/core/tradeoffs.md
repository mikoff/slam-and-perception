---
topic: Architecture Tradeoffs
tags: [mcp, adr, devcontainer, indexing]
related_nodes: [architecture.md, philosophy.md]
---
## TRADEOFFS

- **codebase-memory-mcp** — structural graph indexing.
- **ADR blob compiled from files** — binary requires one 6-section document; `setup.py` concatenates individual files from `decisions/`.
- **Dev Container mounts monorepo root** — ensures `scripts/` and `.agent_knowledge/` are always accessible.
