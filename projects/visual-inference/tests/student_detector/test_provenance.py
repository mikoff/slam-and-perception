from __future__ import annotations

from pathlib import Path
import subprocess

from student_detector.provenance import _project_git_status


def test_project_git_status_ignores_unrelated_monorepo_files(tmp_path: Path) -> None:
    subprocess.run(("git", "init", "-q"), cwd=tmp_path, check=True)
    project = tmp_path / "projects/visual-inference"
    project.mkdir(parents=True)
    (tmp_path / "unrelated.txt").write_text("outside project", encoding="utf-8")
    (project / "changed.py").write_text("inside project", encoding="utf-8")

    status = _project_git_status(tmp_path, project)
    assert status == "?? projects/visual-inference/"
    assert "unrelated.txt" not in status
