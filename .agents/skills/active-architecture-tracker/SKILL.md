---
name: active-architecture-tracker
description: Maintains local, concise ARCHITECTURE.md files within the specific sub-folder where work was done. Overwrites stale context locally, it is an active status document.
---

# Polyglot Local Architecture Tracker

## Purpose
You are responsible for maintaining a local `ARCHITECTURE.md` file inside the specific sub-project directory where code changes occurred.

## Scope & Location Rule
- Find the nearest `ARCHITECTURE.md` in the directory hierarchy of the modified files.
- If none exists in that sub-folder, create one (`<modified-directory>/ARCHITECTURE.md`).
- Do **NOT** update the global repository root unless the change breaks repository-wide architectural contracts.

## Core Directives

1. **In-Place Overwrites (No History):**
   - Directly edit existing sections to reflect the *current reality*.
   - Delete obsolete functions, superseded design patterns, or removed parameters.
   - Do NOT append historical change logs, dates, or version entries.

2. **Domain-Adaptive Documentation:**
   - Adapt the technical descriptions to match the technology in the directory:
     - **C / C++ / Rust:** Focus on memory ownership, header/interface entry points, thread safety, build targets, and performance constraints.
     - **Python / Deep Learning:** Focus on input/output types, data structures, tensor shapes, and framework requirements. You must document tensor dimensions `[B, S, D]`, numerical precision (FP16/BF16/FP32), attention/layer mechanics, GPU memory constraints, and loss/optimizer expectations.
     - **TypeScript / Web:** Focus on state management, API contracts, prop types, and component hierarchies.

3. **Brevity & Scannability:**
   - Keep the file between **40 and 80 lines**.
   - Use concise bullet points and short code blocks over prose.

## Execution Steps

1. Detect the primary sub-directory containing modified files (e.g., `src/cpp_engine/` or `services/api/`).
2. Read or initialize `ARCHITECTURE.md` in that directory.
3. Update the four standard sections:
   - **Module Purpose & Boundaries**
   - **Technical Contracts & Interfaces**
   - **Active Design Patterns & Decisions**
   - **Local Constraints & Gotchas**
4. Save the file back to the local sub-directory.
