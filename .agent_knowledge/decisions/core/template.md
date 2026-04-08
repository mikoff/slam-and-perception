---
topic: [Category]
tags: [tag1, tag2]
related_nodes: [patterns.md]
---

Use one of the 6 ADR sections: PURPOSE, STACK, ARCHITECTURE, PATTERNS, TRADEOFFS, PHILOSOPHY.
Place new files in `.agent_knowledge/decisions/generated/`.
Run `python3 scripts/setup.py` after adding.

Example:

```markdown
---
topic: New Database Choice
tags: [database, persistence]
related_nodes: [stack.md, tradeoffs.md]
---
## PATTERNS

- **SQLite**: Use for local persistence. No external DB server.

## TRADEOFFS

- **SQLite over PostgreSQL** — zero-config, single-file, sufficient for local tooling.
```