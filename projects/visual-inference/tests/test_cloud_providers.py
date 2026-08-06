"""Unit tests for Packet.ai cloud provider integration."""

from __future__ import annotations

import os
import sys
import uuid
from typing import Any
from unittest.mock import patch

# Ensure root of project is in sys.path
proj_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if proj_dir not in sys.path:
    sys.path.insert(0, proj_dir)

import pytest
from scripts.cloud.providers.packet import PacketProvider


def _build_mock_launch_options(pool_specs: list[dict[str, Any]]) -> dict[str, Any]:
    """Helper to generate dynamic launch-options payload with arbitrary pool IDs."""
    pools = []
    for spec in pool_specs:
        pools.append({
            "id": spec.get("id", str(uuid.uuid4())),
            "name": spec.get("name", "test-pool"),
            "gpu_model": spec["gpu_model"],
            "available_gpus": spec.get("available_gpus", 1),
            "price_per_hour": spec.get("price_per_hour", 1.0),
        })
    return {"data": {"pools": pools}}


@patch("scripts.cloud.providers.packet.make_json_request")
def test_packet_resolve_pool_id_universal_matching(mock_make_req: patch) -> None:
    """Verify pool_id resolution dynamically matches pools regardless of pool ID values."""
    id_l40s = f"pool_{uuid.uuid4().hex[:8]}"
    id_a100 = f"pool_{uuid.uuid4().hex[:8]}"
    id_rtx6000 = f"pool_{uuid.uuid4().hex[:8]}"
    id_b200 = f"pool_{uuid.uuid4().hex[:8]}"

    launch_opts = _build_mock_launch_options([
        {"id": id_l40s, "gpu_model": "NVIDIA L40S", "name": "l40s-pool", "available_gpus": 5},
        {"id": id_a100, "gpu_model": "NVIDIA A100 80GB PCIe", "name": "a100-pool", "available_gpus": 2},
        {"id": id_rtx6000, "gpu_model": "NVIDIA RTX PRO 6000 Blackwell", "name": "rtx6000-pool", "available_gpus": 3},
        {"id": id_b200, "gpu_model": "NVIDIA B200", "name": "b200-pool", "available_gpus": 4},
    ])
    mock_make_req.return_value = launch_opts
    provider = PacketProvider(api_key="test_key")

    # 1. Verify exact model matches return the correct dynamic pool IDs
    assert provider._resolve_pool_id("l40s", None, {}) == id_l40s
    assert provider._resolve_pool_id("a100", None, {}) == id_a100
    assert provider._resolve_pool_id("rtx6000", None, {}) == id_rtx6000
    assert provider._resolve_pool_id("b200", None, {}) == id_b200

    # 2. Verify rtx4090 default falls back to budget pool (rtx6000) when no 4090 pool exists
    assert provider._resolve_pool_id("rtx4090", None, {}) == id_rtx6000


@patch("scripts.cloud.providers.packet.make_json_request")
def test_packet_resolve_pool_id_prefers_available_gpus(mock_make_req: patch) -> None:
    """Verify that resolution prefers pools with available_gpus > 0 over full pools."""
    id_empty_pool = f"empty_{uuid.uuid4().hex[:8]}"
    id_active_pool = f"active_{uuid.uuid4().hex[:8]}"

    launch_opts = _build_mock_launch_options([
        {"id": id_empty_pool, "gpu_model": "NVIDIA L40S", "name": "l40s-full", "available_gpus": 0},
        {"id": id_active_pool, "gpu_model": "NVIDIA L40S", "name": "l40s-avail", "available_gpus": 4},
    ])
    mock_make_req.return_value = launch_opts
    provider = PacketProvider(api_key="test_key")

    # Should select the pool with available_gpus > 0 (id_active_pool), not id_empty_pool
    assert provider._resolve_pool_id("l40s", None, {}) == id_active_pool


@patch("scripts.cloud.providers.packet.make_json_request")
def test_packet_resolve_pool_id_explicit_overrides(mock_make_req: patch) -> None:
    """Verify explicit pool_id parameters override dynamic lookup."""
    provider = PacketProvider(api_key="test_key")

    custom_id = f"custom_{uuid.uuid4().hex[:8]}"

    # Extra params override
    assert provider._resolve_pool_id("rtx4090", {"pool_id": custom_id}, {}) == custom_id
    # Kwargs override
    assert provider._resolve_pool_id("rtx4090", None, {"pool_id": custom_id}) == custom_id

    # Environment variable override
    with patch.dict(os.environ, {"CLOUD_POOL_ID": custom_id}):
        assert provider._resolve_pool_id("rtx4090", None, {}) == custom_id


@patch("scripts.cloud.providers.packet.make_json_request")
def test_packet_resolve_pool_id_unknown_gpu_raises_value_error(mock_make_req: patch) -> None:
    """Verify unknown GPU type raises an informative ValueError."""
    launch_opts = _build_mock_launch_options([
        {"gpu_model": "NVIDIA L40S", "name": "l40s-pool"},
    ])
    mock_make_req.return_value = launch_opts
    provider = PacketProvider(api_key="test_key")

    with pytest.raises(ValueError, match="Could not resolve Packet.ai pool_id for GPU type 'nonexistent_gpu'"):
        provider._resolve_pool_id("nonexistent_gpu", None, {})
