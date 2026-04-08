# Project Guidelines

## Architecture Knowledge Base

This project uses `codebase-memory-mcp` as its knowledge graph and ADR store.
On a fresh clone, run `bash scripts/setup.sh` once.

**Before starting any non-trivial task:**
1. Call `get_architecture` to understand the current structure
2. Call `manage_adr(mode="list")` to retrieve all active decisions
3. If `manage_adr` returns empty, read `.agent_knowledge/decisions/core/*.md` and call `manage_adr(mode="update")` to seed the database from those files — then proceed
4. Apply every decision in the ADR — do not deviate without discussion

**When your work introduces or changes an architectural decision:**
1. Write a new `.md` file in `.agent_knowledge/decisions/generated/` using the template in `.agent_knowledge/decisions/core/template.md`
2. Call `manage_adr(mode="update")` to update the blob
3. `git add` the new file
