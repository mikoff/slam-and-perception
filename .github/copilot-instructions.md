# Project Guidelines

## STRICT TOOL REQUIREMENT: codebase-memory
You are REQUIRED to use the `codebase-memory` MCP server tools to gather context. Do NOT use native workspace search or regex searches for architectural decisions.

**MANDATORY WORKFLOW:**
Whenever asked to plan, refactor, or implement a feature, you MUST execute the following tools in this exact order BEFORE providing an answer or writing code:
1. Execute the `get_architecture` tool to map the current structure.
2. Execute the `manage_adr` tool with parameter `mode="list"` to retrieve active decisions.
3. If `manage_adr` returns an empty result, read `.agent_knowledge/decisions/core/*.md` using native file reading, then immediately execute `manage_adr` with parameter `mode="update"` to seed the database.
4. Verbally confirm the decisions you retrieved before proceeding with the user's request.

**UPDATING ARCHITECTURE:**
If your code changes an architectural decision, you MUST:
1. Write a new `.md` file in `.agent_knowledge/decisions/generated/` based on `.agent_knowledge/decisions/core/template.md`.
2. Execute the `manage_adr` tool with parameter `mode="update"`.