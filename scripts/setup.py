#!/usr/bin/env python3
"""Index the repo and seed AGENTS.md as the ADR in codebase-memory-mcp. Called by setup.sh."""

import json
import pathlib
import subprocess

BINARY = "codebase-memory-mcp"
REPO_ROOT = pathlib.Path(__file__).parent.parent
ADR_SOURCE = REPO_ROOT / "AGENTS.md"


def run(args: list[str], check: bool = False) -> str:
    result = subprocess.run([BINARY, *args], capture_output=True, text=True)
    if check and result.returncode != 0:
        raise RuntimeError(f"{BINARY} {' '.join(args[:2])} failed:\n{result.stderr.strip()}")
    return result.stdout.strip()


def run_tool(tool: str, *args: str) -> dict:
    return json.loads(run(["cli", tool, *args], check=True))


def main() -> None:
    print(run(["--version"]))
    run(["config", "set", "auto_index", "true"])

    print("Indexing repository...")
    result = run_tool("index_repository", "--repo-path", str(REPO_ROOT))
    print(f"  nodes={result.get('nodes', '?')}  edges={result.get('edges', '?')}")

    if not ADR_SOURCE.exists():
        raise RuntimeError(f"{ADR_SOURCE} not found")

    print(f"Seeding ADR from {ADR_SOURCE.name}...")
    run_tool(
        "manage_adr",
        "--project", result["project"],
        "--mode", "update",
        "--content", ADR_SOURCE.read_text().strip(),
    )

    print("Done. Reload VS Code.")


if __name__ == "__main__":
    main()
