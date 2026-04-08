#!/usr/bin/env python3
"""Index the repo and seed ADRs into codebase-memory-mcp. Called by setup.sh."""

import json
import pathlib
import re
import subprocess

BINARY = "codebase-memory-mcp"
REPO_ROOT = pathlib.Path(__file__).parent.parent
DECISIONS_ROOT = REPO_ROOT / ".agent_knowledge" / "decisions"


def run(args: list[str], check: bool = False) -> str:
    result = subprocess.run([BINARY, *args], capture_output=True, text=True)
    if check and result.returncode != 0:
        raise RuntimeError(f"{BINARY} {' '.join(args[:2])} failed:\n{result.stderr.strip()}")
    return result.stdout.strip()


def get_project_name() -> str:
    raw = run(["cli", "list_projects"])
    data = json.loads(json.loads(raw)["content"][0]["text"])
    return data["projects"][0]["name"]


def load_decisions() -> str:
    """Concatenate decision files, stripping YAML frontmatter."""
    blocks = []
    for subdir in ("core", "generated"):
        d = DECISIONS_ROOT / subdir
        if not d.exists():
            continue
        for f in sorted(d.glob("*.md")):
            if f.name == "template.md":
                continue
            text = f.read_text()
            text = re.sub(r"^---.*?---\s*", "", text, flags=re.DOTALL)
            blocks.append(text.strip())
    return "\n\n".join(blocks)


def main() -> None:
    print(run(["--version"]))
    run(["config", "set", "auto_index", "true"])

    print("Indexing repository...")
    output = run(["cli", "index_repository", json.dumps({"repo_path": str(REPO_ROOT)})], check=True)
    result = json.loads(json.loads(output)["content"][0]["text"])
    print(f"  nodes={result.get('nodes', '?')}  edges={result.get('edges', '?')}")

    print("Seeding ADRs...")
    project = result.get("project") or get_project_name()
    payload = json.dumps({"mode": "update", "project": project, "content": load_decisions()})
    run(["cli", "manage_adr", payload])

    print("Done. Reload VS Code.")


if __name__ == "__main__":
    main()
