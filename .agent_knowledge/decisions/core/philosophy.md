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
