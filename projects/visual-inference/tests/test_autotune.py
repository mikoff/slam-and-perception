"""Unit tests for dynamic VRAM and batch size autotuning."""

from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock, patch

# Ensure root of project is in sys.path
proj_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if proj_dir not in sys.path:
    sys.path.insert(0, proj_dir)

import torch
from student_detector.autotune import autotune_optimal_batch_size, benchmark_batch_size
from student_detector.config import load_phase3_config


from pathlib import Path

def test_autotune_cpu_fallback(tmp_path) -> None:
    config = load_phase3_config(Path("configs/phase3_attnres.yaml"))
    device = torch.device("cpu")

    batch_size, acc_steps = autotune_optimal_batch_size(config, device)
    assert batch_size == config.data.batch_size
    assert acc_steps == config.schedule.accumulation_steps


@patch("student_detector.autotune.benchmark_batch_size")
@patch("torch.cuda.is_available", return_value=True)
@patch("torch.cuda.get_device_name", return_value="NVIDIA RTX PRO 6000")
@patch("torch.cuda.get_device_properties")
def test_autotune_selects_max_fps_with_headroom(
    mock_props: MagicMock,
    mock_name: MagicMock,
    mock_avail: MagicMock,
    mock_benchmark: MagicMock,
) -> None:
    mock_prop = MagicMock()
    mock_prop.total_memory = 96 * (1024**3)  # 96 GB VRAM
    mock_props.return_value = mock_prop

    # Mock benchmarks for candidate batch sizes [16, 32, 64]
    mock_benchmark.side_effect = [
        {"physical_batch_size": 16, "examples_per_second": 100.0, "vram_headroom_fraction": 0.80, "peak_vram_mb": 5000},
        {"physical_batch_size": 32, "examples_per_second": 220.0, "vram_headroom_fraction": 0.60, "peak_vram_mb": 10000},
        {"physical_batch_size": 64, "examples_per_second": 350.0, "vram_headroom_fraction": 0.30, "peak_vram_mb": 20000},
    ]

    config = load_phase3_config(Path("configs/phase3_attnres.yaml"))
    device = torch.device("cuda:0")

    best_batch, acc_steps = autotune_optimal_batch_size(config, device, candidate_batches=[16, 32, 64])

    assert best_batch == 64
    assert acc_steps == 1  # 64 / 64 = 1
