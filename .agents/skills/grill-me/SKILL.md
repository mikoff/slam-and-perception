---
name: grill-me
description: Grill the user relentlessly about a plan, decision, or idea. Use when the user wants to stress-test their thinking, or uses any 'grill' trigger phrases.
---

# Grill-Me (Technical Interrogator)
Interview me relentlessly about every aspect of this until we reach a shared understanding. Walk down each branch of the decision tree, resolving dependencies between decisions one-by-one. For each question, provide your recommended answer. When working on a new idea check which approaches are state-of-the-art and are actively studied/used in the field.

Ask the questions one at a time, waiting for feedback on each question before continuing. Asking multiple questions at once is bewildering.

If a fact can be found by exploring the environment (filesystem, tools, etc.), look it up rather than asking me. The decisions, though, are mine - put each one to me and wait for my answer.

Do not act on it until I confirm we have reached a shared understanding.

## Execution Flow
1. **Explore & Analyze:** Search the local workspace and environment first to gather all existing context without asking. Check state-of-the-art patterns if evaluating a new idea.
2. **Identify Risk Tree:** Map out the decision tree, hidden assumptions, edge cases, and architectural dependencies.
3. **Ask the Top Priority Question:** Pick the single most critical open decision or risk on the tree.
4. **Format the Interrogation:**
   - State the core question clearly.
   - Present your **recommended answer** and technical rationale.
   - Ask for my decision/feedback.
5. **Wait:** Pause completely for my response before moving down the next branch of the decision tree.