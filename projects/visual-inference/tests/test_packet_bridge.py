from __future__ import annotations

import json
import subprocess
from datetime import UTC, datetime, timedelta
from pathlib import Path
from typing import Any

import pytest
import yaml

from scripts.cloud.packet_bridge import (
    DstackClient,
    DstackRunStartError,
    PacketApiError,
    PacketClient,
    PacketCoordinator,
    PacketHostPrerequisiteError,
    RunState,
)

TEST_PUBLIC_KEY = "ssh-ed25519 test-public-material"


def _coordinator(
    packet: Any,
    dstack: Any,
    store: Any,
    *,
    identity_file: Path,
    public_key: str = TEST_PUBLIC_KEY,
) -> PacketCoordinator:
    return PacketCoordinator(
        packet,
        dstack,
        store,
        identity_file=identity_file,
        public_key=public_key,
    )


class _Store:
    def __init__(self) -> None:
        self.states: dict[str, RunState] = {}

    def get(self, run_id: str) -> RunState | None:
        return self.states.get(run_id)

    def put(self, state: RunState) -> None:
        self.states[state.run_id] = state

    def list(self) -> list[RunState]:
        return list(self.states.values())


class _Packet:
    def __init__(self) -> None:
        self.launched = 0
        self.launched_names: list[str] = []
        self.bootstrap_scripts: list[str] = []
        self.terminated: list[str] = []
        self.terminated_names: list[str] = []
        self.sleeps: list[float] = []
        self.placement_selection_indexes: list[int] = []

    def sleep(self, seconds: float) -> None:
        self.sleeps.append(seconds)

    def launch(
        self,
        name: str,
        gpu: str,
        storage: str | None,
        ssh_key_id: str | None,
        *,
        placement_selection_index: int = 0,
    ) -> dict[str, str]:
        assert ssh_key_id
        self.placement_selection_indexes.append(placement_selection_index)
        self.launched += 1
        self.launched_names.append(name)
        return {"id": f"instance-{self.launched}", "name": name}

    def wait_connection(self, instance_id: str) -> dict[str, object]:
        return {
            "host": instance_id,
            "port": 22,
            "username": "test-user",
            "status": "active",
        }

    def wait_ssh_ready(
        self,
        connection: dict[str, object],
        identity_file: Path,
        *,
        required_disk_bytes: int = 0,
    ) -> None:
        assert connection["status"] == "active"
        assert identity_file
        assert required_disk_bytes >= 0

    def bootstrap_host(
        self,
        connection: dict[str, object],
        identity_file: Path,
        script: str,
    ) -> None:
        assert connection["status"] == "active"
        assert identity_file
        self.bootstrap_scripts.append(script)

    def terminate(self, instance_id: str) -> None:
        self.terminated.append(instance_id)

    def terminate_named(self, name: str) -> None:
        self.terminated_names.append(name)


class _Dstack:
    def __init__(
        self,
        statuses: list[tuple[str | None, str]],
        *,
        register_errors: list[Exception] | None = None,
        submit_errors: list[Exception] | None = None,
    ) -> None:
        self.statuses = statuses
        self.register_errors = register_errors or []
        self.submit_errors = submit_errors or []
        self.submitted = 0
        self.deleted: list[str] = []
        self.registered: list[str] = []
        self.stopped: list[str] = []
        self.verified_fleets: list[str] = []

    def register_fleet(self, **values: object) -> None:
        if self.register_errors:
            raise self.register_errors.pop(0)
        name = str(values["name"])
        self.registered.append(name)

    def submit(self, _state: RunState) -> None:
        self.submitted += 1
        if self.submit_errors:
            raise self.submit_errors.pop(0)

    def verify_fleet_offer(self, name: str) -> None:
        self.verified_fleets.append(name)

    def run_status(self, _name: str) -> tuple[str | None, str]:
        return self.statuses.pop(0)

    def wait_until_started(self, _name: str) -> str:
        return "running"

    def stop_run(self, name: str) -> None:
        self.stopped.append(name)

    def delete_fleet(self, name: str) -> None:
        self.deleted.append(name)


def _state() -> RunState:
    now = datetime.now(UTC).isoformat()
    state = RunState("r1", "A100", "configs/phase3.yaml", "d1", "smoke", now, now)
    state.ssh_key_id = "test-key"
    return state


def test_host_loss_replaces_capacity_but_application_failure_does_not(
    tmp_path: Path,
) -> None:
    packet = _Packet()
    store = _Store()
    dstack = _Dstack(
        [("failed", "SSH host unreachable"), ("failed", "process exited 2")]
    )
    coordinator = _coordinator(packet, dstack, store, identity_file=tmp_path / "key")
    state = _state()
    coordinator.launch_attempt(state)
    assert state.attempts == 1
    assert packet.launched_names == ["vi-packet-r1-1"]
    assert state.fleet_name == "packet-r1-1"
    coordinator.reconcile(state, {state.instance_id: {}})
    assert state.attempts == 2
    assert packet.terminated == ["instance-1"]
    assert packet.launched_names == ["vi-packet-r1-1", "vi-packet-r1-2"]
    assert state.fleet_name == "packet-r1-2"
    coordinator.reconcile(state, {state.instance_id: {}})
    assert state.status == "failed"
    assert state.attempts == 2
    assert packet.terminated == ["instance-1", "instance-2"]


def test_coordinator_loads_packet_host_bootstrap(tmp_path: Path) -> None:
    packet = _Packet()
    public_key = "ssh-ed25519 test-public-material"
    coordinator = _coordinator(
        packet,
        _Dstack([]),
        _Store(),
        identity_file=tmp_path / "key",
        public_key=public_key,
    )

    coordinator.launch_attempt(_state())

    script = packet.bootstrap_scripts[0]
    assert script is not None
    assert public_key not in script
    assert "install_key root /root" in script
    assert "AllowTcpForwarding yes" in script
    assert "bootstrap_dir=/var/lib/visual-inference-bootstrap" in script
    assert "bootstrap_status=$bootstrap_dir/status" in script
    assert "download.docker.com/linux/ubuntu" in script
    assert "nvidia-ctk runtime configure --runtime=docker" in script
    assert '"default-shm-size": "32G"' in script
    assert 'dockerd --validate --config-file "$daemon_config"' in script
    assert "__ENCODED_PUBLIC_KEY__" not in script
    assert "__DOCKER_VERSION__" not in script
    assert "__NVIDIA_TOOLKIT_VERSION__" not in script
    syntax = subprocess.run(
        ["sh", "-n"], input=script, text=True, capture_output=True, check=False
    )
    assert syntax.returncode == 0, syntax.stderr


def test_missing_host_and_orphan_are_cleaned_up(tmp_path: Path) -> None:
    packet = _Packet()
    store = _Store()
    coordinator = _coordinator(
        packet, _Dstack([]), store, identity_file=tmp_path / "key"
    )
    state = _state()
    state.instance_id = "missing"
    state.fleet_name = "packet-r1"
    state.attempts = 1
    coordinator.reconcile(state, {})
    assert state.instance_id == "instance-1"
    assert "missing" in packet.terminated

    old = (datetime.now(UTC) - timedelta(hours=2)).isoformat()
    coordinator.sweep_orphans(
        [state], [{"id": "orphan", "name": "vi-packet-old", "createdAt": old}]
    )
    assert "orphan" in packet.terminated


def test_packet_resolves_gpu_pool_and_available_region() -> None:
    client = PacketClient("secret")
    assert client.api_url == "https://dash.packet.ai"
    client._request = lambda *_args: {  # type: ignore[method-assign]
        "pools": [
            {"id": "l40", "gpu_model": "L40S", "available": True},
            {
                "id": "blackwell",
                "name": "NVIDIA RTX PRO 6000 Blackwell",
                "available_gpus": 2,
            },
            {
                "id": "ada",
                "name": "NVIDIA RTX 6000 Ada",
                "available_gpus": 4,
            },
            {
                "id": "premium2",
                "name": "NVIDIA GeForce RTX 4090 Premium2",
                "region_id": 91,
                "available_gpus": 3,
            },
        ],
        "products": [
            {
                "poolIds": ["blackwell"],
                "regions": [
                    {"id": 3, "name": "California", "available_gpus": 2},
                    {"id": 4, "name": "Virginia", "available_gpus": 1},
                ],
            },
            {"poolIds": ["premium2"], "serviceId": "service-4090"},
        ],
        "regions": [
            {"id": 70, "region_name": "Las Vegas", "city": "Las Vegas"},
            {"id": 78, "region_name": "Dallas", "city": "Dallas"},
            {"id": 91, "region_name": "Dallas-2", "city": "Texas"},
        ],
        "serviceRegions": {
            "service-4090": [
                {"id": 1, "name": "Dallas", "available": False},
                {"id": 2, "name": "Dallas2", "available": True},
            ]
        },
    }
    assert client.resolve_placement("L40S") == ("l40", None)
    assert client.resolve_placement("RTX6000") == ("blackwell", 3)
    assert client.resolve_placement("RTX6000", placement_selection_index=1) == (
        "blackwell",
        4,
    )
    assert client.resolve_placement("RTX4090") == ("premium2", 91)
    assert client.resolve_placement("RTX4090", placement_selection_index=2) == (
        "premium2",
        91,
    )
    with pytest.raises(ValueError, match="no exact pool"):
        client.resolve_placement("A100")


def test_packet_launch_uses_documented_payload() -> None:
    captured: dict[str, object] = {}
    client = PacketClient("secret")
    client.resolve_placement = lambda _gpu, **_options: ("pool-id", 17)  # type: ignore[method-assign]

    def request(
        method: str, path: str, payload: dict[str, object] | None = None
    ) -> dict[str, str]:
        captured.update(method=method, path=path, payload=payload)
        return {"id": "instance-id"}

    client._request = request  # type: ignore[method-assign]

    client.launch(
        "attempt-name",
        "GPU",
        ssh_key_id="key-id",
    )

    assert captured["method"] == "POST"
    assert captured["path"] == "/api/v1/instances"
    payload = captured["payload"]
    assert payload == {
        "name": "attempt-name",
        "pool_id": "pool-id",
        "vgpus": 1,
        "region_id": 17,
        "ssh_key_ids": ["key-id"],
    }


def test_packet_does_not_replay_ambiguous_launch_post() -> None:
    calls = 0

    def opener(*_args: object, **_kwargs: object) -> object:
        nonlocal calls
        calls += 1
        raise TimeoutError("launch response timed out")

    client = PacketClient("secret", attempts=5, opener=opener)

    with pytest.raises(PacketApiError, match="timed out"):
        client._request("POST", "/api/v1/instances", {"name": "attempt"})

    assert calls == 1


def test_packet_connection_requires_active_nested_endpoint() -> None:
    client = PacketClient("secret")
    host = "packet-host.example"
    client._request = lambda *_args: {  # type: ignore[method-assign]
        "status": "pending",
        "connection": {"ip": host, "port": 2200, "username": "remote-user"},
    }
    with pytest.raises(RuntimeError, match="status=pending"):
        client.connection("instance-id")

    client._request = lambda *_args: {  # type: ignore[method-assign]
        "status": "active",
        "connection": {"ip": host, "port": 2200, "username": "remote-user"},
    }
    assert client.connection("instance-id") == {
        "host": host,
        "port": 2200,
        "username": "remote-user",
        "status": "active",
    }


def test_packet_ssh_preflight_uses_exact_connection_and_key(tmp_path: Path) -> None:
    calls: list[tuple[list[str], str]] = []

    def run(command: list[str], **options: object) -> subprocess.CompletedProcess[str]:
        calls.append((command, str(options["input"])))
        return subprocess.CompletedProcess(command, 0, "preflight passed\n", "")

    client = PacketClient("secret", run=run)
    identity = tmp_path / "identity"
    connection = {
        "host": "packet-host.example",
        "port": 2200,
        "username": "remote-user",
    }

    client.wait_ssh_ready(connection, identity)

    command, script = calls[0]
    assert str(identity) in command
    assert "2200" in command
    assert "remote-user@packet-host.example" in command
    assert "NVIDIA Container Toolkit" in script
    assert "__REQUIRED_DISK" not in script
    assert "required_kib=0" in script


def test_packet_ssh_preflight_renders_manifest_disk_requirement(
    tmp_path: Path,
) -> None:
    scripts: list[str] = []

    def run(command: list[str], **options: object) -> subprocess.CompletedProcess[str]:
        scripts.append(str(options["input"]))
        return subprocess.CompletedProcess(command, 0, "preflight passed\n", "")

    client = PacketClient("secret", run=run)
    client.wait_ssh_ready(
        {"host": "packet-host.example", "port": 22, "username": "remote-user"},
        tmp_path / "identity",
        required_disk_bytes=65 * 1024**3,
    )

    assert "required_kib=68157440" in scripts[0]
    assert "need 65 GiB" in scripts[0]
    assert "DockerRootDir" in scripts[0]


def test_packet_ssh_preflight_reports_host_prerequisite_failure(
    tmp_path: Path,
) -> None:
    def run(command: list[str], **_options: object) -> subprocess.CompletedProcess[str]:
        return subprocess.CompletedProcess(
            command,
            20,
            "",
            "dstack host preflight failed: systemd is not running\n",
        )

    client = PacketClient("secret", run=run)
    with pytest.raises(RuntimeError, match="systemd is not running"):
        client.wait_ssh_ready(
            {"host": "packet-host.example", "port": 2200, "username": "remote-user"},
            tmp_path / "identity",
        )


def test_packet_bootstrap_runs_over_exact_ssh_endpoint(tmp_path: Path) -> None:
    calls: list[tuple[list[str], dict[str, object]]] = []

    def run(command: list[str], **options: object) -> subprocess.CompletedProcess[str]:
        calls.append((command, options))
        return subprocess.CompletedProcess(command, 0)

    client = PacketClient("secret", run=run)
    client.bootstrap_host(
        {"host": "packet-host.example", "port": 2200, "username": "remote-user"},
        tmp_path / "identity",
        "bootstrap-script",
    )

    assert calls[0][0][-1] == "true"
    command, options = calls[1]
    assert str(tmp_path / "identity") in command
    assert "2200" in command
    assert "remote-user@packet-host.example" in command
    assert command[-1] == "sudo -n sh -s"
    assert options["input"] == "bootstrap-script"
    assert "capture_output" not in options


def test_packet_bootstrap_retries_ssh_transport(tmp_path: Path) -> None:
    results = iter(
        [
            subprocess.CompletedProcess([], 255),
            subprocess.CompletedProcess([], 0),
            subprocess.CompletedProcess([], 0),
        ]
    )
    sleeps: list[float] = []

    def run(
        _command: list[str], **_options: object
    ) -> subprocess.CompletedProcess[str]:
        return next(results)

    client = PacketClient("secret", run=run, sleep=sleeps.append)
    client.bootstrap_host(
        {"host": "packet-host.example", "port": 2200, "username": "remote-user"},
        tmp_path / "identity",
        "bootstrap-script",
        attempts=2,
        interval=3,
    )

    assert sleeps == [3]


def test_packet_bootstrap_failure_is_explicit(tmp_path: Path) -> None:
    results = iter(
        [
            subprocess.CompletedProcess([], 0),
            subprocess.CompletedProcess([], 32),
            subprocess.CompletedProcess([], 0),
        ]
    )

    def run(command: list[str], **_options: object) -> subprocess.CompletedProcess[str]:
        result = next(results)
        return subprocess.CompletedProcess(command, result.returncode)

    client = PacketClient("secret", run=run)
    with pytest.raises(PacketHostPrerequisiteError, match="exit code 32"):
        client.bootstrap_host(
            {"host": "packet-host.example", "port": 2200, "username": "remote-user"},
            tmp_path / "identity",
            "bootstrap-script",
        )


def test_host_prerequisite_failure_is_not_retried_on_fresh_gpu(
    tmp_path: Path,
) -> None:
    class _IncompatiblePacket(_Packet):
        def wait_ssh_ready(
            self,
            connection: dict[str, object],
            identity_file: Path,
            *,
            required_disk_bytes: int = 0,
        ) -> None:
            assert required_disk_bytes >= 0
            raise PacketHostPrerequisiteError(
                "dstack host preflight failed: systemd is not running"
            )

    packet = _IncompatiblePacket()
    store = _Store()
    coordinator = _coordinator(
        packet, _Dstack([]), store, identity_file=tmp_path / "key"
    )
    state = _state()

    with pytest.raises(PacketHostPrerequisiteError, match="systemd is not running"):
        coordinator.launch_attempt(state)

    assert state.status == "failed"
    assert packet.terminated == ["instance-1"]
    assert packet.launched == 1


def test_nonretryable_packet_failure_is_recorded_as_failed(tmp_path: Path) -> None:
    class _BrokenPacket(_Packet):
        def launch(
            self,
            name: str,
            gpu: str,
            storage: str | None,
            ssh_key_id: str | None,
            *,
            placement_selection_index: int = 0,
        ) -> dict[str, str]:
            raise PacketApiError(404, "wrong endpoint", retryable=False)

    store = _Store()
    coordinator = _coordinator(
        _BrokenPacket(), _Dstack([]), store, identity_file=tmp_path / "key"
    )
    state = _state()

    with pytest.raises(PacketApiError, match="wrong endpoint"):
        coordinator.launch_attempt(state)

    assert state.status == "failed"
    assert state.last_error == "wrong endpoint"
    assert store.get(state.run_id) is state


def test_foreground_submit_retries_transient_launch_with_fresh_name(
    tmp_path: Path,
) -> None:
    class _TransientPacket(_Packet):
        def launch(
            self,
            name: str,
            gpu: str,
            storage: str | None,
            ssh_key_id: str | None,
            *,
            placement_selection_index: int = 0,
        ) -> dict[str, str]:
            self.placement_selection_indexes.append(placement_selection_index)
            self.launched_names.append(name)
            if len(self.launched_names) == 1:
                raise PacketApiError(500, "Packet INTERNAL_ERROR", retryable=True)
            assert ssh_key_id
            self.launched += 1
            return {"id": f"instance-{self.launched}", "name": name}

    packet = _TransientPacket()
    store = _Store()
    coordinator = _coordinator(
        packet, _Dstack([]), store, identity_file=tmp_path / "key"
    )
    state = _state()

    coordinator.submit_with_retries(state, retry_interval=3)

    assert state.status == "submitted"
    assert state.attempts == 2
    assert packet.launched_names == [
        "vi-packet-r1-1",
        "vi-packet-r1-2",
    ]
    assert packet.terminated_names == ["vi-packet-r1-1"]
    assert packet.sleeps == [3]
    assert packet.placement_selection_indexes == [0, 1]


def test_cleanup_uncertainty_stops_fresh_capacity_retry(tmp_path: Path) -> None:
    class _UncertainPacket(_Packet):
        def launch(
            self,
            name: str,
            gpu: str,
            storage: str | None,
            ssh_key_id: str | None,
            *,
            placement_selection_index: int = 0,
        ) -> dict[str, str]:
            raise PacketApiError(500, "Packet INTERNAL_ERROR", retryable=True)

        def terminate_named(self, name: str) -> None:
            raise PacketApiError(500, "cleanup unavailable", retryable=True)

    state = _state()
    coordinator = _coordinator(
        _UncertainPacket(), _Dstack([]), _Store(), identity_file=tmp_path / "key"
    )

    with pytest.raises(PacketApiError, match="INTERNAL_ERROR"):
        coordinator.submit_with_retries(state, retry_interval=0)

    assert state.status == "failed"
    assert "cleanup unavailable" in str(state.last_error)
    assert state.attempts == 1


def test_stale_provisioning_without_instance_launches_fresh_attempt(
    tmp_path: Path,
) -> None:
    store = _Store()
    packet = _Packet()
    coordinator = _coordinator(
        packet, _Dstack([]), store, identity_file=tmp_path / "key"
    )
    state = _state()
    state.status = "provisioning"
    state.attempts = 1
    state.updated_at = (datetime.now(UTC) - timedelta(minutes=31)).isoformat()

    coordinator.reconcile(state, {})

    assert state.status == "submitted"
    assert state.attempts == 2
    assert packet.terminated_names == ["vi-packet-r1-1"]
    assert packet.launched_names == ["vi-packet-r1-2"]


def test_ambiguous_launch_is_destroyed_instead_of_adopted(tmp_path: Path) -> None:
    class _AmbiguousPacket(_Packet):
        def launch(
            self,
            name: str,
            gpu: str,
            storage: str | None,
            ssh_key_id: str | None,
            *,
            placement_selection_index: int = 0,
        ) -> dict[str, str]:
            self.launched_names.append(name)
            raise RuntimeError("Packet launch did not return an instance ID")

    store = _Store()
    dstack = _Dstack([])
    packet = _AmbiguousPacket()
    coordinator = _coordinator(packet, dstack, store, identity_file=tmp_path / "key")
    state = _state()

    with pytest.raises(RuntimeError, match="did not return"):
        coordinator.launch_attempt(state)

    assert state.instance_id is None
    assert state.status == "failed"
    assert packet.launched_names == ["vi-packet-r1-1"]
    assert packet.terminated_names == ["vi-packet-r1-1"]
    assert dstack.submitted == 0


def test_ssh_fleet_does_not_mix_backend_nodes_with_ssh_config(
    tmp_path: Path,
) -> None:
    client = DstackClient()
    applied: list[tuple[dict[str, object], dict[str, object]]] = []
    client._apply = lambda config, **options: applied.append((config, options))  # type: ignore[method-assign]

    client.register_fleet(
        name="fleet",
        host="host",
        port=22,
        user="user",
        identity_file=tmp_path / "key",
    )

    config, options = applied[0]
    assert "nodes" not in config
    assert "ssh_config" in config
    assert "detach" not in options


def test_packet_task_is_fully_rendered_for_exact_fleet(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = DstackClient()
    applied: list[tuple[dict[str, Any], dict[str, Any]]] = []
    client._apply = lambda config, **options: applied.append((config, options))  # type: ignore[method-assign]
    monkeypatch.setattr(
        "scripts.cloud.packet_bridge.subprocess.run",
        lambda *_args, **_options: subprocess.CompletedProcess(
            [], 0, "git@example.com:team/repository.git\n", ""
        ),
    )
    state = _state()
    state.source_commit = "a" * 40
    state.fleet_name = "packet-r1-1"

    client.submit(state)

    config, options = applied[0]
    assert config["name"] == "r1"
    assert config["resources"]["gpu"] == "1"
    assert config["resources"]["cpu"] == "1.."
    assert config["resources"]["memory"] == "16GB.."
    assert config["resources"]["disk"] == "1GB.."
    assert config["tags"]["run_id"] == "r1"
    assert config["fleets"] == ["packet-r1-1"]
    assert "instances" not in config
    assert "backends" not in config
    assert config["creation_policy"] == "reuse"
    assert config["retry"] is False
    assert config["spot_policy"] == "auto"
    assert "max_price" not in config
    assert config["repos"] == [
        {
            "url": "git@example.com:team/repository.git",
            "hash": "a" * 40,
            "path": "/dstack/repo",
        }
    ]
    assert "${{ env." not in yaml.safe_dump(config).lower()
    assert options["detach"] is True
    assert options["environment"]["DSTACK_GPU"] == "A100"


def test_runpod_task_renders_selected_gpu_and_keeps_backend_retry(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = DstackClient()
    applied: list[dict[str, Any]] = []
    client._apply = lambda config, **_options: applied.append(config)  # type: ignore[method-assign]
    monkeypatch.setattr(
        "scripts.cloud.packet_bridge.subprocess.run",
        lambda *_args, **_options: subprocess.CompletedProcess(
            [], 0, "https://example.com/team/repository.git\n", ""
        ),
    )
    state = _state()
    state.gpu = "L40S"
    state.source_commit = "b" * 40

    client.submit(state)

    config = applied[0]
    assert config["resources"]["gpu"] == "L40S:1"
    assert config["backends"] == ["runpod"]
    assert config["retry"] == {
        "on_events": ["no-capacity", "interruption"],
        "duration": "24h",
    }
    assert "fleets" not in config
    assert "creation_policy" not in config
    assert "instances" not in config
    assert config["repos"] == [
        {
            "url": "https://example.com/team/repository.git",
            "hash": "b" * 40,
            "path": "/dstack/repo",
        }
    ]


def test_detached_task_waits_until_running(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = DstackClient()
    statuses = iter(
        [
            (None, "run not found"),
            ("submitted", "waiting for scheduler"),
            ("running", ""),
        ]
    )
    monkeypatch.setattr(client, "run_status", lambda _name: next(statuses))
    sleeps: list[float] = []
    monkeypatch.setattr("scripts.cloud.packet_bridge.time.sleep", sleeps.append)

    assert (
        client.wait_until_started("r1", attempts=3, interval=2, stable_checks=1)
        == "running"
    )
    assert sleeps == [2, 2]


def test_run_status_reads_pinned_dstack_json_schema(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payload = {
        "project": "cloud-training",
        "runs": [
            {
                "run_spec": {"run_name": "r1"},
                "status": "running",
                "status_message": "running",
                "latest_job_submission": {"status_message": "pull complete"},
            }
        ],
    }
    monkeypatch.setattr(
        "scripts.cloud.packet_bridge.subprocess.run",
        lambda *_args, **_options: subprocess.CompletedProcess(
            [], 0, json.dumps(payload), ""
        ),
    )

    assert DstackClient().run_status("r1") == ("running", "pull complete")


def test_run_status_keeps_top_level_name_compatibility(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payload = {"runs": [{"name": "r1", "status": "done"}]}
    monkeypatch.setattr(
        "scripts.cloud.packet_bridge.subprocess.run",
        lambda *_args, **_options: subprocess.CompletedProcess(
            [], 0, json.dumps(payload), ""
        ),
    )

    assert DstackClient().run_status("r1") == ("done", "")


def test_detached_task_failure_surfaces_dstack_reason(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = DstackClient()
    monkeypatch.setattr(
        client,
        "run_status",
        lambda _name: ("failed", "No fleets match requested resources"),
    )

    with pytest.raises(DstackRunStartError, match="No fleets match"):
        client.wait_until_started("r1", attempts=1, interval=0)


def test_transient_running_state_must_survive_stabilization(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = DstackClient()
    statuses = iter(
        [
            ("running", ""),
            ("failed", "process exited 2"),
        ]
    )
    monkeypatch.setattr(client, "run_status", lambda _name: next(statuses))
    monkeypatch.setattr("scripts.cloud.packet_bridge.time.sleep", lambda _seconds: None)

    with pytest.raises(DstackRunStartError, match="process exited 2"):
        client.wait_until_started("r1", attempts=2, interval=0, stable_checks=2)


def test_running_at_scheduling_deadline_gets_full_stabilization_window(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = DstackClient()
    statuses = iter(
        [
            ("submitted", "waiting for scheduler"),
            ("provisioning", "pulling"),
            ("running", "running"),
            ("running", "running"),
            ("running", "running"),
        ]
    )
    monkeypatch.setattr(client, "run_status", lambda _name: next(statuses))
    sleeps: list[float] = []
    monkeypatch.setattr("scripts.cloud.packet_bridge.time.sleep", sleeps.append)

    assert (
        client.wait_until_started("r1", attempts=3, interval=2, stable_checks=3)
        == "running"
    )
    assert sleeps == [2, 2, 2, 2]


def test_submit_rejects_dstack_no_offer_success_exit(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = DstackClient()
    client._apply = lambda *_args, **_options: (  # type: ignore[method-assign]
        "No matching instance offers available.\nRun r1 submitted, detaching...\n"
    )
    monkeypatch.setattr(
        "scripts.cloud.packet_bridge.subprocess.run",
        lambda *_args, **_options: subprocess.CompletedProcess(
            [], 0, "https://example.com/team/repository.git\n", ""
        ),
    )
    state = _state()
    state.source_commit = "a" * 40
    state.fleet_name = "packet-r1-1"

    with pytest.raises(DstackRunStartError, match="no matching offer"):
        client.submit(state)


def test_fleet_offer_check_identifies_rejecting_requirement(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = DstackClient()
    counts = iter([1, 0])
    monkeypatch.setattr(client, "_offer_count", lambda *_args: next(counts))

    with pytest.raises(DstackRunStartError, match="offer check runtime-memory"):
        client.verify_fleet_offer("packet-r1-1", attempts=1, interval=0)


def test_post_provision_failure_cleans_up_before_fresh_attempt(
    tmp_path: Path,
) -> None:
    store = _Store()
    dstack = _Dstack([], register_errors=[RuntimeError("SSH registration failed")])
    packet = _Packet()
    coordinator = _coordinator(packet, dstack, store, identity_file=tmp_path / "key")
    state = _state()

    with pytest.raises(RuntimeError, match="SSH registration failed"):
        coordinator.launch_attempt(state)

    assert state.status == "desired"
    assert state.instance_id is None
    assert state.fleet_name is None
    assert packet.terminated == ["instance-1"]
    assert dstack.deleted == ["packet-r1-1"]

    coordinator.reconcile(state, {})

    assert state.instance_id == "instance-2"
    assert state.status == "submitted"
    assert packet.launched_names == ["vi-packet-r1-1", "vi-packet-r1-2"]
    assert dstack.registered == ["packet-r1-2"]
    assert dstack.submitted == 1


def test_submit_cli_error_monitors_task_when_dstack_accepted_it(tmp_path: Path) -> None:
    store = _Store()
    dstack = _Dstack(
        [("running", "")],
        submit_errors=[RuntimeError("dstack CLI connection closed")],
    )
    packet = _Packet()
    coordinator = _coordinator(packet, dstack, store, identity_file=tmp_path / "key")
    state = _state()

    coordinator.launch_attempt(state)

    assert state.status == "submitted"
    assert state.dstack_run_name == "r1"
    assert state.instance_id == "instance-1"
    assert packet.terminated == []


def test_startup_gate_error_never_destroys_a_running_task(tmp_path: Path) -> None:
    store = _Store()
    dstack = _Dstack([("running", "running")])

    def fail_start(_name: str) -> str:
        raise DstackRunStartError("local startup observation window expired")

    dstack.wait_until_started = fail_start  # type: ignore[method-assign]
    packet = _Packet()
    coordinator = _coordinator(packet, dstack, store, identity_file=tmp_path / "key")
    state = _state()

    coordinator.launch_attempt(state)

    assert state.status == "submitted"
    assert state.dstack_run_name == "r1"
    assert dstack.stopped == []
    assert dstack.deleted == []
    assert packet.terminated == []


def test_submit_cli_error_does_not_hide_terminal_dstack_run(tmp_path: Path) -> None:
    store = _Store()
    dstack = _Dstack(
        [("failed", "No fleets match requested resources")],
        submit_errors=[RuntimeError("dstack CLI connection closed")],
    )
    packet = _Packet()
    coordinator = _coordinator(packet, dstack, store, identity_file=tmp_path / "key")
    state = _state()

    with pytest.raises(DstackRunStartError, match="No fleets match"):
        coordinator.launch_attempt(state)

    assert state.status == "failed"
    assert dstack.stopped == ["r1"]
    assert packet.terminated == ["instance-1"]


def test_submit_failure_without_dstack_task_destroys_attempt(tmp_path: Path) -> None:
    store = _Store()
    dstack = _Dstack(
        [(None, "run not found")],
        submit_errors=[RuntimeError("invalid task configuration")],
    )
    packet = _Packet()
    coordinator = _coordinator(packet, dstack, store, identity_file=tmp_path / "key")
    state = _state()

    with pytest.raises(RuntimeError, match="invalid task configuration"):
        coordinator.launch_attempt(state)

    assert state.status == "failed"
    assert state.instance_id is None
    assert state.fleet_name is None
    assert packet.terminated == ["instance-1"]
    assert dstack.deleted == ["packet-r1-1"]


def test_task_that_fails_before_start_is_not_treated_as_accepted(
    tmp_path: Path,
) -> None:
    store = _Store()
    dstack = _Dstack([("failed", "No fleets match requested resources")])

    def fail_start(_name: str) -> str:
        raise DstackRunStartError(
            "dstack run r1 ended before startup: status=failed; "
            "reason=No fleets match requested resources"
        )

    dstack.wait_until_started = fail_start  # type: ignore[method-assign]
    packet = _Packet()
    coordinator = _coordinator(packet, dstack, store, identity_file=tmp_path / "key")
    state = _state()

    with pytest.raises(DstackRunStartError, match="No fleets match"):
        coordinator.launch_attempt(state)

    assert state.status == "failed"
    assert state.instance_id is None
    assert dstack.stopped == ["r1"]
    assert dstack.deleted == ["packet-r1-1"]
    assert packet.terminated == ["instance-1"]


def test_stale_provisioning_with_submitted_task_is_only_monitored(
    tmp_path: Path,
) -> None:
    store = _Store()
    dstack = _Dstack([("running", "")])
    packet = _Packet()
    coordinator = _coordinator(packet, dstack, store, identity_file=tmp_path / "key")
    state = _state()
    state.status = "provisioning"
    state.attempts = 1
    state.instance_id = "existing-instance"
    state.fleet_name = "packet-r1-1"
    state.updated_at = (datetime.now(UTC) - timedelta(minutes=31)).isoformat()

    coordinator.reconcile(state, {state.instance_id: {}})

    assert state.status == "running"
    assert state.instance_id == "existing-instance"
    assert packet.launched == 0
    assert dstack.registered == []
    assert dstack.submitted == 0


def test_stale_provisioning_without_task_replaces_server(tmp_path: Path) -> None:
    store = _Store()
    dstack = _Dstack([(None, "run not found")])
    packet = _Packet()
    coordinator = _coordinator(packet, dstack, store, identity_file=tmp_path / "key")
    state = _state()
    state.status = "provisioning"
    state.attempts = 1
    state.instance_id = "existing-instance"
    state.fleet_name = "packet-r1-1"
    state.updated_at = (datetime.now(UTC) - timedelta(minutes=31)).isoformat()

    coordinator.reconcile(state, {state.instance_id: {}})

    assert packet.terminated == ["existing-instance"]
    assert packet.launched_names == ["vi-packet-r1-2"]
    assert dstack.deleted == ["packet-r1-1"]
    assert dstack.registered == ["packet-r1-2"]
    assert state.status == "submitted"


def test_terminal_legacy_state_is_cleaned_without_reuse(tmp_path: Path) -> None:
    store = _Store()
    dstack = _Dstack([])
    packet = _Packet()
    coordinator = _coordinator(packet, dstack, store, identity_file=tmp_path / "key")
    state = _state()
    state.status = "failed"
    state.attempts = 2
    state.instance_id = "existing-instance"
    state.fleet_name = "packet-r1"

    coordinator.reconcile(state, {state.instance_id: {}})

    assert packet.terminated == ["existing-instance"]
    assert packet.launched == 0
    assert dstack.deleted == ["packet-r1"]
    assert state.status == "failed"


def test_reconciler_does_not_race_recent_provisioning(tmp_path: Path) -> None:
    store = _Store()
    dstack = _Dstack([("running", "")])
    coordinator = _coordinator(_Packet(), dstack, store, identity_file=tmp_path / "key")
    state = _state()
    state.status = "provisioning"
    state.instance_id = "existing-instance"

    coordinator.reconcile(state, {state.instance_id: {}})

    assert dstack.statuses == [("running", "")]
    assert dstack.submitted == 0
