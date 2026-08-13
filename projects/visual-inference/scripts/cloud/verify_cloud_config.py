"""Static supply-chain and recovery-policy checks for cloud configuration."""

from __future__ import annotations

import re
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
REPOSITORY_ROOT = PROJECT_ROOT.parents[1]
SHA_ACTION = re.compile(r"uses:\s+[\w./-]+@[0-9a-f]{40}(?:\s|$)")


def main() -> None:
    task_path = PROJECT_ROOT / "cloud-training.dstack.yml"
    task_text = task_path.read_text(encoding="utf-8")
    task = yaml.safe_load(task_text)
    if "${{ env." in task_text.lower():
        raise ValueError(
            "dstack task must contain concrete values; arbitrary environment "
            "interpolation is unsupported"
        )
    image = str(task.get("image", ""))
    if "@sha256:" not in image:
        raise ValueError("cloud training image must be pinned by digest")
    retry_events = set(task.get("retry", {}).get("on_events", []))
    if retry_events != {"no-capacity", "interruption"}:
        raise ValueError("dstack must not retry application errors")
    if task.get("max_duration") in {None, "off"}:
        raise ValueError("cloud task requires a finite max_duration")
    if task.get("working_dir") != "/dstack/repo/projects/visual-inference":
        raise ValueError("cloud task must run from the cloned monorepo project")
    commands = task.get("commands", [])
    if not commands or "pyproject.toml" not in str(commands[0]):
        raise ValueError("cloud task must verify its project working directory first")
    if "/dev/shm" not in str(commands[0]):
        raise ValueError("cloud task must verify its effective shared-memory mount")
    if task.get("resources", {}).get("shm_size") != "32GB":
        raise ValueError("cloud task must request 32GB shared memory from dstack")

    workflows = sorted((REPOSITORY_ROOT / ".github/workflows").glob("*.yml"))
    for workflow in workflows:
        text = workflow.read_text(encoding="utf-8")
        for line in text.splitlines():
            if "uses:" in line and SHA_ACTION.search(line) is None:
                raise ValueError(f"unpinned action in {workflow}: {line.strip()}")
    cloud_text = (
        REPOSITORY_ROOT / ".github/workflows/packet-visual-inference.yml"
    ).read_text(encoding="utf-8")
    for forbidden in (
        "extra_training_args",
        "extra_params",
        "remote_entrypoint.sh",
        "run_cloud_daemon.py",
    ):
        if forbidden in cloud_text:
            raise ValueError(f"obsolete unsafe cloud surface remains: {forbidden}")
    if "dstack apply -f cloud-training.dstack.yml" in cloud_text:
        raise ValueError("workflow bypasses the validated dstack task renderer")
    if "scripts/cloud/submit_dstack_task.py" not in cloud_text:
        raise ValueError("RunPod dispatch must use the validated dstack task renderer")
    print("Cloud workflow and dstack policy verified")


if __name__ == "__main__":
    main()
