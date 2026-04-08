---
topic: Agent Philosophy
tags: [workflow, agent, rules]
related_nodes: [patterns.md, tradeoffs.md]
---
## PHILOSOPHY

- Call `get_architecture` and `manage_adr(mode="list")` before any non-trivial task.
- Do not deviate from decisions without discussion.
- New decisions go in `.agent_knowledge/decisions/generated/`, then run `setup.py`.
- Follow all patterns in PATTERNS — they are mandatory, not suggestions.
