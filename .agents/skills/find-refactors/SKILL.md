---
name: find-refactors
description: Analyze current code changes and nearby architecture to identify evidence-backed, behavior-preserving refactoring opportunities.
disable-model-invocation: true
---

# Find Refactoring Candidates

Analyze the active changes first, then use repository history to identify
high-leverage architectural improvements.

## Scope

- Honor any files, paths, or commit range specified by the user.
- Otherwise inspect:
  - staged and unstaged changes;
  - relevant untracked files;
  - commits on the current branch relative to its likely base branch.
- Read repository guidance such as `AGENTS.md` before evaluating the design.
- Do not modify code.
- Ignore generated, vendored, lockfile, and formatting-only changes unless relevant.
- Prefer improvements near the active changes over unrelated redesigns.

## Analysis

1. **Understand the Change**
   - Identify the behavior being added or modified.
   - Trace relevant entry points, callers, dependencies, tests, and state ownership.
   - Establish the existing module boundaries and intended responsibilities.

2. **Find Architectural Friction**
   Look for:
   - shotgun surgery or repeated co-changes across boundaries;
   - shallow wrappers and interfaces that expose implementation complexity;
   - duplicated policy or validation across modules;
   - mixed responsibilities and unclear state ownership;
   - hidden side effects, global state, or excessive test mocking;
   - dependency cycles or unstable dependency direction;
   - abstractions that make the current change harder than necessary.

3. **Validate Each Candidate**
   - Cite exact files, symbols, and relevant history.
   - Explain how the evidence demonstrates recurring friction.
   - Distinguish architectural problems from ordinary implementation complexity.
   - Trace affected callers and tests.
   - Describe the behavior that must remain unchanged.
   - Reject speculative or low-payoff candidates.

## Output

Return at most three ranked candidates. For each include:

- **Where:** Files, symbols, and boundary involved.
- **Evidence:** Concrete observations from the diff, code, tests, or history.
- **Problem:** Why the current structure creates recurring friction.
- **Proposed Change:** The new responsibility and module boundary.
- **Payoff:** What becomes simpler or more stable.
- **Cost/Risk:** Migration scope and likely failure modes.
- **Validation:** Tests or checks needed to preserve behavior.

Prefer a smaller local refactor when it provides most of the benefit. Explicitly
say when no strong refactoring candidate is supported by the evidence.

Finish by asking which candidate I want to stress-test with `/grill-me`.