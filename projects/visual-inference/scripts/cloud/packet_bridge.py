"""Packet.ai capacity bridge for dstack SSH-fleet training tasks.

Packet owns VM lifecycle; dstack owns task execution. Desired state is durable
in S3 so a scheduled reconciler can recover after a GitHub runner disconnects.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import random
import subprocess
import tempfile
import time
import urllib.error
import urllib.request
from dataclasses import asdict, dataclass, fields
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Callable, Protocol

import yaml

if __package__:
    from .dataset_staging import AwsCli, download_manifest, required_staging_bytes
else:
    from dataset_staging import AwsCli, download_manifest, required_staging_bytes

PROJECT_ROOT = Path(__file__).resolve().parents[2]
TASK_CONFIG = PROJECT_ROOT / "cloud-training.dstack.yml"
PACKET_HOST_BOOTSTRAP = Path(__file__).with_name("packet_host_bootstrap.sh")
PACKET_PREFIX = "vi-packet-"
STATE_PREFIX = "control/packet-runs"
TERMINAL_DSTACK_SUCCESS = {"done", "finished", "succeeded", "success"}
TERMINAL_DSTACK_FAILURE = {"failed", "terminated", "aborted", "error"}
STARTING_DSTACK_STATUSES = {
    "pending",
    "submitted",
    "provisioning",
    "pulling",
    "running",
}
PROVISIONING_LEASE_SECONDS = 30 * 60
READY_PACKET_STATUSES = {"active", "running"}
PACKET_BOOTSTRAP_LOG = "/var/log/visual-inference-bootstrap.log"
DOCKER_ENGINE_VERSION = "29.6.1"
NVIDIA_CONTAINER_TOOLKIT_VERSION = "1.19.1-1"
PACKET_DISK_RESERVE_BYTES = 64 * 1024**3
PACKET_GPU_MATCH_TERMS = {
    # Workflow label -> Packet's current names for the 96 GB Blackwell SKU.
    # Do not let the shorter label select the older RTX 6000 Ada by accident.
    "rtx6000": ("rtxpro6000", "rtx6000pro"),
}


class DstackRunStartError(RuntimeError):
    """A submitted dstack run failed or timed out before starting."""


DSTACK_HOST_PREFLIGHT = r"""set -eu
fail() { echo "dstack host preflight failed: $1" >&2; exit 20; }
command -v sudo >/dev/null || fail "sudo is not installed"
sudo -n true >/dev/null 2>&1 || fail "the SSH user lacks passwordless sudo"
sudo test -r /var/lib/visual-inference-bootstrap/status || \
  fail "Packet host bootstrap did not create its status file"
bootstrap_status=$(sudo cat /var/lib/visual-inference-bootstrap/status)
case "$bootstrap_status" in
  ready) ;;
  failed:*)
    sudo tail -n 100 /var/log/visual-inference-bootstrap.log >&2 || true
    fail "Packet host bootstrap reported $bootstrap_status"
    ;;
  *) fail "Packet host bootstrap reported unknown status: $bootstrap_status" ;;
esac
command -v curl >/dev/null || fail "curl is not installed"
command -v systemctl >/dev/null || fail "systemd/systemctl is not installed"
sudo systemctl show-environment >/dev/null 2>&1 || fail "systemd is not running"
command -v docker >/dev/null || fail "Docker is not installed"
sudo docker info >/dev/null 2>&1 || fail "the Docker daemon is not usable"
sudo jq -e '.["default-shm-size"] == "32G"' /etc/docker/daemon.json \
  >/dev/null 2>&1 || fail "Docker does not have the 32G shared-memory default"
command -v nvidia-smi >/dev/null || fail "the NVIDIA driver tools are not installed"
nvidia-smi -L >/dev/null 2>&1 || fail "nvidia-smi cannot see a GPU"
[ "$(nproc)" -ge 8 ] || fail "fewer than 8 CPU cores are available"
awk '$1 == "MemTotal:" {exit !($2 >= 33554432)}' /proc/meminfo || \
  fail "less than 32 GiB RAM is available"
docker_root=$(sudo docker info --format '{{.DockerRootDir}}')
available_kib=$(df -Pk "$docker_root" | awk 'NR == 2 {print $4}')
required_kib=__REQUIRED_DISK_KIB__
if [ "$available_kib" -lt "$required_kib" ]; then
  echo "Docker/data filesystem diagnostics:" >&2
  df -hT "$docker_root" >&2 || true
  findmnt -T "$docker_root" >&2 || true
  fail "insufficient Docker/data disk: need __REQUIRED_DISK_GIB__ GiB, have $((available_kib / 1048576)) GiB"
fi
command -v nvidia-container-cli >/dev/null || \
  fail "NVIDIA Container Toolkit CLI is not installed"
sudo nvidia-container-cli info >/dev/null 2>&1 || \
  fail "NVIDIA Container Toolkit cannot access the GPU driver"
sudo docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -qi nvidia || \
  fail "NVIDIA Container Toolkit is not registered with Docker"
sudo sshd -T 2>/dev/null | grep -Eq '^allowtcpforwarding (yes|all)$' || \
  fail "sshd does not allow TCP forwarding"
test -w "$HOME/.ssh/authorized_keys" || \
  fail "$HOME/.ssh/authorized_keys is not writable by the SSH user"
echo "dstack host preflight passed"
"""


def _utc_now() -> str:
    return datetime.now(UTC).isoformat()


def _packet_bootstrap_script(public_key: str) -> str:
    """Bootstrap Packet's Ubuntu image for dstack and expose durable status."""
    encoded_key = base64.b64encode(public_key.encode()).decode()
    template = PACKET_HOST_BOOTSTRAP.read_text(encoding="utf-8")
    return (
        template.replace("__ENCODED_PUBLIC_KEY__", encoded_key)
        .replace("__DOCKER_VERSION__", DOCKER_ENGINE_VERSION)
        .replace("__NVIDIA_TOOLKIT_VERSION__", NVIDIA_CONTAINER_TOOLKIT_VERSION)
    )


@dataclass
class RunState:
    run_id: str
    gpu: str
    config: str
    dataset_id: str
    mode: str
    created_at: str
    updated_at: str
    source_commit: str = ""
    display_name: str = ""
    batch_candidates: str = "16,32,64,96,128"
    ssh_key_id: str = ""
    status: str = "desired"
    attempts: int = 0
    max_attempts: int = 3
    instance_id: str | None = None
    fleet_name: str | None = None
    dstack_run_name: str | None = None
    last_error: str | None = None
    required_disk_bytes: int = 0

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "RunState":
        allowed = {field.name for field in fields(cls)}
        return cls(**{key: item for key, item in value.items() if key in allowed})


class PacketApiError(RuntimeError):
    def __init__(self, status: int | None, message: str, *, retryable: bool) -> None:
        super().__init__(message)
        self.status = status
        self.retryable = retryable


class PacketHostPrerequisiteError(RuntimeError):
    """The reachable Packet image cannot satisfy dstack's SSH host contract."""


class PacketClient:
    """Current Packet REST API with bounded transport retries."""

    def __init__(
        self,
        api_key: str,
        *,
        api_url: str = "https://dash.packet.ai",
        attempts: int = 5,
        opener: Callable[..., Any] = urllib.request.urlopen,
        sleep: Callable[[float], None] = time.sleep,
        run: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
    ) -> None:
        self.api_key = api_key
        self.api_url = api_url.rstrip("/")
        self.attempts = attempts
        self.opener = opener
        self.sleep = sleep
        self.run = run

    def _request(
        self,
        method: str,
        path: str,
        payload: dict[str, Any] | None = None,
    ) -> Any:
        url = f"{self.api_url}{path}"
        body = json.dumps(payload).encode() if payload is not None else None
        request = urllib.request.Request(
            url,
            data=body,
            method=method,
            headers={
                "X-API-Key": self.api_key,
                "Content-Type": "application/json",
                "User-Agent": "visual-inference-packet-bridge/1",
            },
        )
        # A failed instance-creation response is ambiguous: Packet may have
        # created capacity before its API returned 5xx. Do not blindly replay
        # POST. The coordinator first removes the exact attempt name and then
        # starts a new, uniquely named attempt.
        request_attempts = 1 if method == "POST" else self.attempts
        for attempt in range(request_attempts):
            try:
                with self.opener(request, timeout=30) as response:
                    raw = response.read()
                decoded = json.loads(raw) if raw else {}
                return decoded.get("data", decoded)
            except urllib.error.HTTPError as error:
                retryable = error.code == 429 or 500 <= error.code < 600
                details = error.read().decode("utf-8", errors="replace")
                failure = PacketApiError(
                    error.code,
                    f"Packet {method} {path} returned {error.code}: {details}",
                    retryable=retryable,
                )
            except (urllib.error.URLError, TimeoutError, OSError) as error:
                failure = PacketApiError(None, str(error), retryable=True)
            if not failure.retryable or attempt + 1 == request_attempts:
                raise failure
            delay = min(2**attempt + random.random(), 30.0)
            print(
                f"Transient Packet {method} {path} failure "
                f"({attempt + 1}/{request_attempts}): {failure}; "
                f"retrying in {delay:.1f}s",
                flush=True,
            )
            self.sleep(delay)
        raise AssertionError("unreachable")

    def list_instances(self) -> list[dict[str, Any]]:
        value = self._request("GET", "/api/v1/instances")
        return list(value) if isinstance(value, list) else []

    @staticmethod
    def _normalized_gpu_name(value: object) -> str:
        return "".join(
            character for character in str(value).lower() if character.isalnum()
        )

    @staticmethod
    def _available_gpu_count(pool: dict[str, Any]) -> int:
        # Current Packet OpenAPI defines available_gpus per physical pool. It is
        # authoritative when present; a legacy/general `available: true` must
        # not make an empty Dallas pool win over a non-empty Dallas2 pool.
        if pool.get("available_gpus") is not None:
            try:
                return max(int(pool["available_gpus"]), 0)
            except (TypeError, ValueError):
                return 0
        explicit = pool.get("available")
        if isinstance(explicit, bool):
            return int(explicit)
        if isinstance(explicit, (int, float)):
            return max(int(explicit), 0)
        try:
            return 1 if str(explicit).strip().lower() in {"true", "yes", "1"} else 0
        except (TypeError, ValueError):
            return 0

    @classmethod
    def _region_candidates(
        cls, options: dict[str, Any], pool: dict[str, Any]
    ) -> list[dict[str, Any]]:
        """Collect region choices exposed by current and dashboard API shapes."""
        candidates: dict[int, dict[str, Any]] = {}

        def add(values: object) -> None:
            if isinstance(values, list):
                for value in values:
                    add(value)
                return
            if not isinstance(values, dict) or values.get("id") is None:
                return
            try:
                region_id = int(values["id"])
            except (TypeError, ValueError):
                return
            candidates[region_id] = {**candidates.get(region_id, {}), **values}

        # In Packet's live response, top-level regions are a global catalog and
        # each deployable pool carries its own authoritative region_id. Never
        # interpret the entire catalog as compatible with a selected pool.
        top_regions = {
            int(region["id"]): region
            for region in options.get("regions", [])
            if isinstance(region, dict)
            and region.get("id") is not None
            and str(region["id"]).isdigit()
        }
        if pool.get("region_id") is not None:
            region_id = int(pool["region_id"])
            region = {
                **top_regions.get(region_id, {}),
                "id": region_id,
                "available_gpus": cls._available_gpu_count(pool),
            }
            if pool.get("region_name"):
                region["region_name"] = pool["region_name"]
            return [region]

        add(pool.get("regions"))
        add(pool.get("region"))

        pool_id = str(pool.get("id", ""))
        products = options.get("products", [])
        related_products = []
        for product in products if isinstance(products, list) else []:
            if not isinstance(product, dict):
                continue
            product_pool_ids = product.get("poolIds", product.get("pool_ids", []))
            if pool_id in {str(value) for value in product_pool_ids or []}:
                related_products.append(product)
                add(product.get("regions"))

        service_regions = options.get(
            "serviceRegions", options.get("service_regions", {})
        )
        if isinstance(service_regions, dict):
            for product in related_products:
                service_id = product.get("serviceId", product.get("service_id"))
                if service_id is not None:
                    add(
                        service_regions.get(
                            str(service_id), service_regions.get(service_id)
                        )
                    )

        return list(candidates.values())

    @classmethod
    def _region_available(cls, region: dict[str, Any]) -> bool:
        if region.get("available_gpus") is not None:
            return cls._available_gpu_count(region) > 0
        available = region.get("available")
        if available is None:
            # Product/service region lists contain compatible choices only.
            return True
        return cls._available_gpu_count(region) > 0

    def resolve_placement(
        self, gpu: str, *, placement_selection_index: int = 0
    ) -> tuple[str, int | None]:
        value = self._request("GET", "/api/v1/launch-options")
        options = value if isinstance(value, dict) else {}
        pools = options.get("pools", [])
        requested = self._normalized_gpu_name(gpu)
        match_terms = PACKET_GPU_MATCH_TERMS.get(requested, (requested,))
        matches = []
        for pool in pools:
            text = self._normalized_gpu_name(
                " ".join(
                    str(pool.get(key, ""))
                    for key in ("gpu_model", "name", "product_name")
                )
            )
            if any(term in text for term in match_terms):
                matches.append(pool)
        if not matches:
            raise ValueError(f"Packet has no exact pool for requested GPU {gpu!r}")
        capacities = [(pool, self._available_gpu_count(pool)) for pool in matches]
        print(
            f"Packet pool candidates for {gpu}: "
            + ", ".join(
                f"{pool.get('name') or pool.get('gpu_model') or pool.get('id')}="
                f"{count} GPU(s)"
                for pool, count in capacities
            ),
            flush=True,
        )
        available = [(pool, count) for pool, count in capacities if count > 0]
        if not available:
            names = ", ".join(
                str(pool.get("name") or pool.get("gpu_model") or pool.get("id"))
                for pool in matches
            )
            raise ValueError(
                f"Packet has no available capacity for requested GPU {gpu!r}; "
                f"matching pool(s): {names}"
            )
        # Packet's public launch surface binds a deployable pool to region_id.
        # Rotate complete valid placements, never pool and region independently.
        selected, available_count = available[
            placement_selection_index % len(available)
        ]
        if not selected.get("id"):
            raise ValueError(f"Packet pool for requested GPU {gpu!r} has no ID")
        print(
            "Selected Packet pool "
            f"{selected['id']} ({selected.get('name') or selected.get('gpu_model')}) "
            f"for {gpu}; aggregate reported capacity={available_count}",
            flush=True,
        )

        regions = self._region_candidates(options, selected)
        if not regions:
            print(
                "Packet launch options expose no region candidates; using "
                "Packet's automatic region selection",
                flush=True,
            )
            return str(selected["id"]), None
        available_regions = [
            region for region in regions if self._region_available(region)
        ]
        print(
            f"Packet region candidates for {gpu}: "
            + ", ".join(
                f"{region.get('region_name') or region.get('name') or region.get('city') or region['id']}="
                f"{region.get('available_gpus', region.get('available', 'compatible'))}"
                for region in regions
            ),
            flush=True,
        )
        if not available_regions:
            raise ValueError(
                f"Packet has no available region for requested GPU {gpu!r} "
                f"in pool {selected['id']}"
            )
        available_regions.sort(
            key=lambda region: (
                -self._available_gpu_count(region)
                if region.get("available_gpus") is not None
                else 0,
                self._normalized_gpu_name(
                    region.get("name") or region.get("city") or ""
                ),
                int(region["id"]),
            )
        )
        # A pool's own region_id yields exactly one choice. If it is absent,
        # only explicitly product/service-linked regions reach this list and
        # may be rotated; the unrelated global catalog never reaches it.
        region = available_regions[placement_selection_index % len(available_regions)]
        region_id = int(region["id"])
        print(
            "Selected Packet region "
            f"{region_id} ({region.get('region_name') or region.get('name') or region.get('city')}) for {gpu}; "
            f"pool_candidate={placement_selection_index % len(available) + 1}/"
            f"{len(available)}, region_candidate="
            f"{placement_selection_index % len(available_regions) + 1}/"
            f"{len(available_regions)}",
            flush=True,
        )
        return str(selected["id"]), region_id

    @staticmethod
    def _ssh_key_matches(
        key: dict[str, Any], public_key: str, fingerprint: str
    ) -> bool:
        """Match the different SSH-key representations returned by Packet."""
        for field in ("publicKey", "public_key"):
            registered = str(key.get(field, "")).strip().split()
            if len(registered) >= 2 and " ".join(registered[:2]) == public_key:
                return True
        registered_fingerprint = str(key.get("fingerprint", "")).strip()
        if registered_fingerprint and registered_fingerprint == fingerprint:
            return True
        preview = str(key.get("keyPreview") or key.get("key_preview") or "").strip()
        preview_prefix = preview.split("...", 1)[0].split("…", 1)[0].strip()
        return len(preview_prefix) >= 16 and preview_prefix in public_key

    def resolve_ssh_key(
        self,
        public_key: str,
        fingerprint: str,
        explicit_key_id: str | None = None,
    ) -> str:
        """Resolve a pre-registered Packet key without relying on key creation."""
        fields = public_key.strip().split()
        if len(fields) < 2:
            raise ValueError("derived SSH public key is malformed")
        # Comments are optional and may differ between Packet and ssh-keygen.
        # The algorithm and encoded key material uniquely identify the key.
        normalized = " ".join(fields[:2])
        value = self._request("GET", "/api/v1/ssh-keys")
        keys = value if isinstance(value, list) else []

        if explicit_key_id:
            selected = [key for key in keys if str(key.get("id")) == explicit_key_id]
            if not selected:
                raise ValueError(
                    "PACKET_SSH_KEY_ID does not identify a registered Packet SSH key"
                )
            key = selected[0]
            has_key_metadata = any(
                key.get(field)
                for field in (
                    "publicKey",
                    "public_key",
                    "fingerprint",
                    "keyPreview",
                    "key_preview",
                )
            )
            if has_key_metadata and not self._ssh_key_matches(
                key, normalized, fingerprint
            ):
                raise ValueError(
                    "PACKET_SSH_KEY_ID belongs to a different SSH private key"
                )
            return explicit_key_id

        matching = []
        for key in keys:
            if self._ssh_key_matches(key, normalized, fingerprint):
                matching.append(key)
        if len(matching) > 1:
            raise RuntimeError(
                "multiple Packet SSH keys match PACKET_SSH_PRIVATE_KEY; set "
                "PACKET_SSH_KEY_ID explicitly"
            )
        key_id = matching[0].get("id") if matching else None
        if not key_id:
            raise RuntimeError(
                "PACKET_SSH_PRIVATE_KEY has no matching registered public key. "
                "Add its public key in the Packet dashboard and set "
                "PACKET_SSH_KEY_ID to that key's ID. Local fingerprint: "
                f"{fingerprint}"
            )
        return str(key_id)

    def launch(
        self,
        name: str,
        gpu: str,
        storage_block_id: str | None = None,
        ssh_key_id: str | None = None,
        *,
        placement_selection_index: int = 0,
    ) -> dict[str, Any]:
        pool_id, region_id = self.resolve_placement(
            gpu, placement_selection_index=placement_selection_index
        )
        payload: dict[str, Any] = {
            "name": name,
            "pool_id": pool_id,
            "vgpus": 1,
        }
        if region_id is not None:
            payload["region_id"] = region_id
        if not ssh_key_id:
            raise ValueError("Packet launch requires an explicit SSH key ID")
        payload["ssh_key_ids"] = [ssh_key_id]
        if storage_block_id:
            payload["persistent_storage_block_id"] = storage_block_id
        value = self._request("POST", "/api/v1/instances", payload)
        if not isinstance(value, dict):
            raise RuntimeError("Packet launch did not return an instance ID")
        instance_id = value.get("id") or value.get("instance_id")
        if not instance_id:
            raise RuntimeError("Packet launch did not return an instance ID")
        return {**value, "id": str(instance_id)}

    def connection(self, instance_id: str) -> dict[str, Any]:
        value = self._request("GET", f"/api/v1/instances/{instance_id}/connection")
        if not isinstance(value, dict):
            raise RuntimeError("Packet connection response has no host")
        details = value.get("connection", value)
        if not isinstance(details, dict):
            raise RuntimeError("Packet connection response has no host")
        host = details.get("host") or details.get("ip")
        if not host:
            raise RuntimeError("Packet connection response has no host")
        status = str(value.get("status", "")).lower()
        if status and status not in READY_PACKET_STATUSES:
            raise RuntimeError(f"Packet SSH endpoint is not ready (status={status})")
        return {
            "host": str(host),
            "port": int(details.get("port", 22)),
            "username": str(details.get("username", "ubuntu")),
            "status": status or "unknown",
            **({"password": details["password"]} if details.get("password") else {}),
        }

    def wait_connection(
        self, instance_id: str, *, attempts: int = 90, interval: float = 10.0
    ) -> dict[str, Any]:
        last_error: BaseException | None = None
        for _ in range(attempts):
            try:
                return self.connection(instance_id)
            except (PacketApiError, RuntimeError) as error:
                last_error = error
                self.sleep(interval)
        raise TimeoutError(
            f"Packet instance {instance_id} did not expose SSH in time"
        ) from last_error

    @staticmethod
    def _ssh_command(
        connection: dict[str, Any], identity_file: Path, remote_command: str
    ) -> list[str]:
        destination = f"{connection['username']}@{connection['host']}"
        return [
            "ssh",
            "-F",
            "/dev/null",
            "-i",
            str(identity_file),
            "-p",
            str(connection["port"]),
            "-o",
            "BatchMode=yes",
            "-o",
            "IdentitiesOnly=yes",
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "UserKnownHostsFile=/dev/null",
            "-o",
            "LogLevel=ERROR",
            "-o",
            "ConnectTimeout=10",
            destination,
            remote_command,
        ]

    def bootstrap_host(
        self,
        connection: dict[str, Any],
        identity_file: Path,
        script: str,
        *,
        attempts: int = 12,
        interval: float = 10.0,
    ) -> None:
        """Run the host bootstrap explicitly over Packet's verified SSH endpoint."""
        probe_command = self._ssh_command(connection, identity_file, "true")
        for attempt in range(attempts):
            try:
                probe = self.run(
                    probe_command,
                    text=True,
                    capture_output=True,
                    timeout=30,
                )
            except subprocess.TimeoutExpired:
                probe = subprocess.CompletedProcess(probe_command, 255)
            if probe.returncode == 0:
                break
            if probe.returncode != 255:
                details = (probe.stderr or probe.stdout or "").strip()
                raise PacketHostPrerequisiteError(
                    details or "Packet SSH readiness probe failed"
                )
            if attempt + 1 < attempts:
                print(
                    "Packet SSH authentication is not ready for bootstrap "
                    f"({attempt + 1}/{attempts}); retrying",
                    flush=True,
                )
                self.sleep(interval)
        else:
            raise TimeoutError(
                "Packet SSH authentication did not become ready for bootstrap"
            )

        command = self._ssh_command(connection, identity_file, "sudo -n sh -s")
        try:
            result = self.run(command, input=script, text=True, timeout=15 * 60)
        except subprocess.TimeoutExpired as error:
            raise PacketHostPrerequisiteError(
                "Packet host bootstrap exceeded 15 minutes; inspect "
                f"{PACKET_BOOTSTRAP_LOG}"
            ) from error
        if result.returncode == 0:
            return
        if result.returncode == 255:
            raise TimeoutError("Packet SSH connection dropped during host bootstrap")
        print("Packet bootstrap failed; remote log tail follows:", flush=True)
        tail_command = self._ssh_command(
            connection,
            identity_file,
            f"sudo -n tail -n 100 {PACKET_BOOTSTRAP_LOG}",
        )
        try:
            self.run(tail_command, text=True, timeout=45)
        except subprocess.TimeoutExpired:
            print("Timed out while retrieving the bootstrap log", flush=True)
        raise PacketHostPrerequisiteError(
            "Packet host bootstrap failed over SSH with exit code "
            f"{result.returncode}; inspect {PACKET_BOOTSTRAP_LOG}"
        )

    def wait_ssh_ready(
        self,
        connection: dict[str, Any],
        identity_file: Path,
        *,
        required_disk_bytes: int = 0,
        attempts: int = 6,
        interval: float = 10.0,
    ) -> None:
        destination = f"{connection['username']}@{connection['host']}"
        command = self._ssh_command(connection, identity_file, "sh -s")
        required_disk_kib = (required_disk_bytes + 1023) // 1024
        required_disk_gib = (required_disk_bytes + 1024**3 - 1) // 1024**3
        preflight = DSTACK_HOST_PREFLIGHT.replace(
            "__REQUIRED_DISK_KIB__", str(required_disk_kib)
        ).replace("__REQUIRED_DISK_GIB__", str(required_disk_gib))
        last_transport_error = "SSH did not run"
        for attempt in range(attempts):
            try:
                result = self.run(
                    command,
                    input=preflight,
                    text=True,
                    capture_output=True,
                    timeout=45,
                )
            except subprocess.TimeoutExpired as error:
                last_transport_error = f"ssh timed out after {error.timeout}s"
                if attempt + 1 < attempts:
                    self.sleep(interval)
                    continue
                break
            details = "\n".join(
                part.strip() for part in (result.stdout, result.stderr) if part.strip()
            )
            if result.returncode == 0:
                print(details or "dstack host preflight passed", flush=True)
                return
            if result.returncode != 255:
                raise PacketHostPrerequisiteError(
                    details or "dstack host preflight failed"
                )
            last_transport_error = details or f"ssh exited {result.returncode}"
            if attempt + 1 < attempts:
                self.sleep(interval)
        raise TimeoutError(
            f"Packet SSH authentication never became ready for {destination}: "
            f"{last_transport_error}"
        )

    def terminate(self, instance_id: str) -> None:
        try:
            self._request("DELETE", f"/api/v1/instances/{instance_id}")
        except PacketApiError as error:
            if error.status != 404:
                raise

    def terminate_named(self, name: str) -> None:
        """Delete ambiguous launch results without ever adopting them."""
        for instance in self.list_instances():
            if instance.get("name") == name and instance.get("id"):
                self.terminate(str(instance["id"]))


class StateStore(Protocol):
    def get(self, run_id: str) -> RunState | None: ...
    def put(self, state: RunState) -> None: ...
    def list(self) -> list[RunState]: ...


class S3StateStore:
    def __init__(self, bucket: str, endpoint: str | None = None) -> None:
        self.bucket = bucket
        self.endpoint = endpoint

    def _aws(
        self, *arguments: str, capture: bool = False
    ) -> subprocess.CompletedProcess[str]:
        command = ["aws", *arguments]
        if self.endpoint:
            command.extend(["--endpoint-url", self.endpoint])
        return subprocess.run(command, check=True, text=True, capture_output=capture)

    def _uri(self, run_id: str) -> str:
        return f"s3://{self.bucket}/{STATE_PREFIX}/{run_id}.json"

    def get(self, run_id: str) -> RunState | None:
        result = subprocess.run(
            ["aws", "s3", "cp", self._uri(run_id), "-", "--no-progress"]
            + (["--endpoint-url", self.endpoint] if self.endpoint else []),
            text=True,
            capture_output=True,
        )
        if result.returncode != 0:
            return None
        return RunState.from_dict(json.loads(result.stdout))

    def put(self, state: RunState) -> None:
        state.updated_at = _utc_now()
        with tempfile.NamedTemporaryFile(
            "w", suffix=".json", encoding="utf-8"
        ) as stream:
            json.dump(asdict(state), stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            self._aws("s3", "cp", stream.name, self._uri(state.run_id), "--no-progress")

    def list(self) -> list[RunState]:
        result = self._aws(
            "s3api",
            "list-objects-v2",
            "--bucket",
            self.bucket,
            "--prefix",
            f"{STATE_PREFIX}/",
            "--output",
            "json",
            capture=True,
        )
        values = json.loads(result.stdout).get("Contents", [])
        states = []
        for entry in values:
            key = str(entry.get("Key", ""))
            if key.endswith(".json"):
                if state := self.get(Path(key).stem):
                    states.append(state)
        return states


def _nested_strings(value: Any) -> list[str]:
    if isinstance(value, str):
        return [value]
    if isinstance(value, dict):
        return [item for child in value.values() for item in _nested_strings(child)]
    if isinstance(value, list):
        return [item for child in value for item in _nested_strings(child)]
    return []


class DstackClient:
    def __init__(self, *, task_config: Path = TASK_CONFIG) -> None:
        self.task_config = task_config

    def _apply(
        self,
        config: dict[str, Any],
        *,
        detach: bool = False,
        environment: dict[str, str] | None = None,
    ) -> str:
        with tempfile.NamedTemporaryFile(
            "w", suffix=".dstack.yml", dir=self.task_config.parent, encoding="utf-8"
        ) as stream:
            yaml.safe_dump(config, stream, sort_keys=False)
            stream.flush()
            command = ["dstack", "apply", "-f", stream.name, "-y"]
            if detach:
                command.append("-d")
            process = subprocess.Popen(
                command,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            output: list[str] = []
            assert process.stdout is not None
            for line in process.stdout:
                print(line, end="", flush=True)
                output.append(line)
            return_code = process.wait()
            if return_code:
                details = "".join(output)[-8000:].strip()
                raise RuntimeError(
                    f"dstack apply exited {return_code}"
                    + (f":\n{details}" if details else " without diagnostic output")
                )
            return "".join(output)

    def register_fleet(
        self,
        *,
        name: str,
        host: str,
        port: int,
        user: str,
        identity_file: Path,
    ) -> None:
        self._apply(
            {
                "type": "fleet",
                "name": name,
                "ssh_config": {
                    "user": user,
                    "port": port,
                    "identity_file": str(identity_file),
                    "hosts": [host],
                },
            }
        )

    def submit(self, state: RunState) -> None:
        config = yaml.safe_load(self.task_config.read_text(encoding="utf-8"))
        config["name"] = state.run_id
        resources = config.setdefault("resources", {})
        resources["gpu"] = "1" if state.fleet_name else f"{state.gpu}:1"
        config.setdefault("tags", {})["run_id"] = state.run_id
        if state.fleet_name:
            config.pop("backends", None)
            # The real Packet capacity was checked directly over SSH. Avoid
            # rejecting that known host on redundant marketplace constraints.
            resources.update(cpu="1..", memory="16GB..", disk="1GB..")
            config["spot_policy"] = "auto"
            config.pop("max_price", None)
            # Every attempt owns a unique one-node fleet. Targeting the fleet
            # avoids depending on dstack's generated instance-name convention.
            config["fleets"] = [state.fleet_name]
            config["creation_policy"] = "reuse"
            config["retry"] = False
        environment = os.environ.copy()
        environment.update(
            {
                "RUN_ID": state.run_id,
                "RUN_DISPLAY_NAME": state.display_name,
                "DSTACK_GPU": state.gpu,
                "CONFIG_PATH": state.config,
                "DATASET_ID": state.dataset_id,
                "RUN_MODE": state.mode,
                "BATCH_CANDIDATES": state.batch_candidates,
                "SOURCE_COMMIT": state.source_commit,
            }
        )
        repository = PROJECT_ROOT.parents[1]
        repository_url = subprocess.run(
            ["git", "remote", "get-url", "origin"],
            cwd=repository,
            check=True,
            text=True,
            capture_output=True,
        ).stdout.strip()
        if not repository_url:
            raise RuntimeError("Git origin URL is required for immutable submission")
        if not state.source_commit:
            raise RuntimeError("source commit is required for immutable submission")
        config["repos"] = [
            {
                "url": repository_url,
                "hash": state.source_commit,
                "path": "/dstack/repo",
            }
        ]
        unresolved = [
            value for value in _nested_strings(config) if "${{ env." in value.lower()
        ]
        if unresolved:
            raise ValueError(
                "cloud task contains unsupported dstack environment interpolation: "
                + ", ".join(sorted(set(unresolved)))
            )
        output = self._apply(config, detach=True, environment=environment) or ""
        if "no matching instance offers available" in output.lower():
            raise DstackRunStartError(
                f"dstack found no matching offer in fleet {state.fleet_name}; "
                f"requested resources: {config.get('resources', {})}"
            )

    def run_status(self, run_name: str) -> tuple[str | None, str]:
        result = subprocess.run(
            ["dstack", "ps", "--all", "--json"],
            check=True,
            text=True,
            capture_output=True,
        )
        value = json.loads(result.stdout)
        rows = value if isinstance(value, list) else value.get("runs", [])
        for row in rows:
            run_spec = row.get("run_spec")
            run_spec = run_spec if isinstance(run_spec, dict) else {}
            # dstack 0.20 serializes the configured name inside Run.run_spec.
            # Keep the top-level fallback for compatibility with other versions.
            serialized_name = run_spec.get("run_name") or row.get("name")
            if serialized_name == run_name:
                status = str(row.get("status", "")).lower()
                latest = row.get("latest_job_submission")
                latest = latest if isinstance(latest, dict) else {}
                reason = str(
                    latest.get("error")
                    or latest.get("termination_reason_message")
                    or latest.get("status_message")
                    or row.get("error")
                    or row.get("termination_reason_message")
                    or row.get("status_message")
                    or ""
                )
                return status, reason
        return None, "run not found"

    def _offer_count(self, fleet_name: str, arguments: list[str]) -> int:
        result = subprocess.run(
            [
                "dstack",
                "offer",
                "--fleet",
                fleet_name,
                "--reuse",
                "--max-offers",
                "1",
                "--json",
                *arguments,
            ],
            check=True,
            text=True,
            capture_output=True,
        )
        value = json.loads(result.stdout)
        return int(value.get("total_offers", 0))

    def verify_fleet_offer(
        self,
        fleet_name: str,
        *,
        attempts: int = 7,
        interval: float = 5.0,
    ) -> None:
        """Prove which dstack requirement, if any, rejects the SSH fleet."""
        stages = [
            (
                "baseline",
                [
                    "--gpu",
                    "1",
                    "--cpu",
                    "1..",
                    "--memory",
                    "1GB..",
                    "--disk",
                    "1GB..",
                    "--spot-auto",
                ],
            ),
            (
                "runtime-memory",
                [
                    "--gpu",
                    "1",
                    "--cpu",
                    "1..",
                    "--memory",
                    "16GB..",
                    "--disk",
                    "1GB..",
                    "--spot-auto",
                ],
            ),
        ]
        baseline_count = 0
        for attempt in range(attempts):
            baseline_count = self._offer_count(fleet_name, stages[0][1])
            print(
                f"dstack fleet offer check baseline: {baseline_count} offer(s) "
                f"({attempt + 1}/{attempts})",
                flush=True,
            )
            if baseline_count:
                break
            if attempt + 1 < attempts:
                time.sleep(interval)
        if not baseline_count:
            raise DstackRunStartError(
                f"dstack fleet {fleet_name} exposes no reusable offer even with "
                "minimal resource constraints"
            )
        for stage, arguments in stages[1:]:
            count = self._offer_count(fleet_name, arguments)
            print(f"dstack fleet offer check {stage}: {count} offer(s)", flush=True)
            if not count:
                raise DstackRunStartError(
                    f"dstack fleet {fleet_name} is rejected at offer check "
                    f"{stage}; arguments={arguments}"
                )

    def wait_until_started(
        self,
        run_name: str,
        *,
        attempts: int = 91,
        interval: float = 10.0,
        stable_checks: int = 7,
    ) -> str:
        """Wait for scheduling, then give RUNNING its own stabilization window."""
        last_status: str | None = None
        last_reason = "run not found"
        consecutive_running = 0
        poll_budget = attempts
        poll_index = 0
        while poll_index < poll_budget:
            status, reason = self.run_status(run_name)
            poll_index += 1
            last_status, last_reason = status, reason
            if status in TERMINAL_DSTACK_SUCCESS:
                return status
            if status == "running":
                consecutive_running += 1
                if consecutive_running >= stable_checks:
                    return status
                if consecutive_running == 1:
                    # Pulling a large CUDA image may consume the entire scheduling
                    # window. Do not make the independent runtime observation race
                    # what remains of that window.
                    poll_budget = max(
                        poll_budget,
                        poll_index + stable_checks - 1,
                    )
                    print(
                        f"dstack run {run_name} is running; observing it for "
                        f"{(stable_checks - 1) * interval:.0f}s before detaching",
                        flush=True,
                    )
            else:
                consecutive_running = 0
            if status in TERMINAL_DSTACK_FAILURE:
                raise DstackRunStartError(
                    f"dstack run {run_name} ended before startup: "
                    f"status={status}; reason={reason or 'not reported'}"
                )
            if status is not None and status not in STARTING_DSTACK_STATUSES:
                raise DstackRunStartError(
                    f"dstack run {run_name} returned unknown startup status "
                    f"{status!r}: {reason or 'no diagnostic'}"
                )
            if poll_index == 1 or poll_index % 6 == 0:
                print(
                    f"Waiting for dstack run {run_name} to start "
                    f"({poll_index}/{poll_budget}); "
                    f"status={status or 'not-found'}; "
                    f"reason={reason or 'not reported'}; "
                    f"stable={consecutive_running}/{stable_checks}",
                    flush=True,
                )
            if poll_index < poll_budget:
                time.sleep(interval)
        raise DstackRunStartError(
            f"dstack run {run_name} did not become stably running within "
            f"the {(attempts - 1) * interval:.0f}s scheduling window plus "
            f"{(stable_checks - 1) * interval:.0f}s stabilization window: "
            f"status={last_status or 'not-found'}; "
            f"reason={last_reason or 'not reported'}; "
            f"stable={consecutive_running}/{stable_checks}"
        )

    def stop_run(self, name: str) -> None:
        result = subprocess.run(
            ["dstack", "stop", name, "-x", "-y"],
            text=True,
            capture_output=True,
        )
        details = f"{result.stdout}\n{result.stderr}".strip()
        if result.returncode and not any(
            marker in details.lower() for marker in ("does not exist", "not found")
        ):
            raise RuntimeError(
                f"dstack stop exited {result.returncode}"
                + (f":\n{details}" if details else " without diagnostic output")
            )

    def delete_fleet(self, name: str) -> None:
        result = subprocess.run(
            ["dstack", "fleet", "delete", name, "-y"],
            text=True,
            capture_output=True,
        )
        details = f"{result.stdout}\n{result.stderr}".strip()
        if result.returncode and not any(
            marker in details.lower() for marker in ("does not exist", "not found")
        ):
            raise RuntimeError(
                f"dstack fleet delete exited {result.returncode}"
                + (f":\n{details}" if details else " without diagnostic output")
            )


def _retryable_host_failure(status: str | None, reason: str) -> bool:
    text = f"{status or ''} {reason}".lower()
    return any(
        marker in text
        for marker in (
            "interrupt",
            "unreachable",
            "host",
            "ssh",
            "connection",
            "instance lost",
        )
    )


class PacketCoordinator:
    def __init__(
        self,
        packet: PacketClient,
        dstack: DstackClient,
        store: StateStore,
        *,
        identity_file: Path,
        public_key: str,
        storage_block_id: str | None = None,
    ) -> None:
        self.packet = packet
        self.dstack = dstack
        self.store = store
        self.identity_file = identity_file
        self.bootstrap_script = _packet_bootstrap_script(public_key)
        self.storage_block_id = storage_block_id

    def _cleanup(self, state: RunState) -> None:
        failures = []
        if state.fleet_name:
            try:
                self.dstack.delete_fleet(state.fleet_name)
                state.fleet_name = None
            except Exception as error:
                failures.append(f"fleet {state.fleet_name}: {error}")
        if state.instance_id:
            try:
                self.packet.terminate(state.instance_id)
                state.instance_id = None
            except Exception as error:
                failures.append(f"instance {state.instance_id}: {error}")
        if failures:
            raise RuntimeError("cleanup failed: " + "; ".join(failures))

    def launch_attempt(self, state: RunState) -> None:
        if state.attempts >= state.max_attempts:
            state.status = "failed"
            state.last_error = "maximum Packet replacement attempts exhausted"
            self.store.put(state)
            return
        state.attempts += 1
        if not state.ssh_key_id:
            state.status = "failed"
            state.last_error = "Packet run has no explicit SSH key ID"
            self.store.put(state)
            return
        state.status = "provisioning"
        self.store.put(state)
        instance_name = f"{PACKET_PREFIX}{state.run_id}-{state.attempts}"
        try:
            instance = self.packet.launch(
                instance_name,
                state.gpu,
                self.storage_block_id,
                state.ssh_key_id,
                placement_selection_index=state.attempts - 1,
            )
            state.instance_id = str(instance["id"])
            state.fleet_name = f"packet-{state.run_id}-{state.attempts}"
            self.store.put(state)
            self._finish_submission(state)
        except Exception as error:
            if state.status == "submitting":
                try:
                    status, reason = self.dstack.run_status(state.run_id)
                except Exception as probe_error:
                    state.last_error = (
                        f"{error}; could not determine whether dstack accepted "
                        f"the task: {probe_error}"
                    )
                    self.store.put(state)
                    raise
                if status in TERMINAL_DSTACK_FAILURE:
                    error = DstackRunStartError(
                        f"dstack run {state.run_id} ended before startup: "
                        f"status={status}; reason={reason or 'not reported'}"
                    )
                if status == "running" or status in TERMINAL_DSTACK_SUCCESS:
                    state.dstack_run_name = state.run_id
                    state.status = "submitted"
                    state.last_error = None
                    self.store.put(state)
                    print(
                        f"dstack run {state.run_id} is {status}; preserving the "
                        "healthy task despite a local startup-gate error",
                        flush=True,
                    )
                    return
                if (
                    not isinstance(error, DstackRunStartError)
                    and status is not None
                    and status not in TERMINAL_DSTACK_FAILURE
                ):
                    state.dstack_run_name = state.run_id
                    state.status = "submitted"
                    state.last_error = None
                    self.store.put(state)
                    print(
                        f"dstack accepted {state.run_id} despite a CLI error; "
                        "the reconciler will monitor it",
                        flush=True,
                    )
                    return
                if isinstance(error, DstackRunStartError) and status is not None:
                    try:
                        self.dstack.stop_run(state.run_id)
                    except Exception as stop_error:
                        state.last_error = (
                            f"{error}; could not stop dstack run: {stop_error}"
                        )
                        self.store.put(state)
                        raise
            cleanup_errors = []
            had_instance_id = state.instance_id is not None
            try:
                self._cleanup(state)
            except Exception as cleanup_error:
                cleanup_errors.append(str(cleanup_error))
            if not had_instance_id:
                try:
                    self.packet.terminate_named(instance_name)
                except Exception as cleanup_error:
                    cleanup_errors.append(
                        f"ambiguous instance {instance_name}: {cleanup_error}"
                    )
            state.last_error = str(error)
            if cleanup_errors:
                state.last_error += "; " + "; ".join(cleanup_errors)
            retryable = not cleanup_errors and (
                (isinstance(error, PacketApiError) and error.retryable)
                or isinstance(error, TimeoutError)
                or (
                    not isinstance(error, PacketHostPrerequisiteError)
                    and _retryable_host_failure(None, str(error))
                )
            )
            state.status = (
                "desired"
                if retryable and state.attempts < state.max_attempts
                else "failed"
            )
            self.store.put(state)
            raise error

    def submit_with_retries(
        self,
        state: RunState,
        *,
        retry_interval: float = 60.0,
    ) -> None:
        """Keep initial submission attached while retryable attempts remain."""
        while True:
            try:
                self.launch_attempt(state)
                return
            except Exception:
                if state.status != "desired":
                    raise
                delay = min(retry_interval * (2 ** (state.attempts - 1)), 120.0)
                print(
                    "Packet attempt "
                    f"{state.attempts}/{state.max_attempts} failed transiently "
                    f"and was cleaned up: {state.last_error}. Retrying with a fresh "
                    "instance in "
                    f"{delay:.0f}s",
                    flush=True,
                )
                self.packet.sleep(delay)

    def _finish_submission(self, state: RunState) -> None:
        if not state.instance_id:
            raise ValueError("cannot submit without a Packet instance")
        if not state.fleet_name:
            raise ValueError("cannot submit without an attempt-specific fleet")
        print(
            f"Waiting for Packet SSH connection metadata for {state.instance_id}",
            flush=True,
        )
        connection = self.packet.wait_connection(state.instance_id)
        print(
            "Packet reports ready SSH endpoint "
            f"{connection['username']}@{connection['host']}:{connection['port']} "
            f"(status={connection['status']})",
            flush=True,
        )
        print("Bootstrapping Packet host over SSH", flush=True)
        self.packet.bootstrap_host(
            connection, self.identity_file, self.bootstrap_script
        )
        print("Verifying SSH and dstack host prerequisites", flush=True)
        self.packet.wait_ssh_ready(
            connection,
            self.identity_file,
            required_disk_bytes=state.required_disk_bytes,
        )
        print(
            f"Registering dstack SSH fleet {state.fleet_name}",
            flush=True,
        )
        self.dstack.register_fleet(
            name=state.fleet_name,
            host=str(connection["host"]),
            port=int(connection.get("port", 22)),
            user=str(connection.get("username", "ubuntu")),
            identity_file=self.identity_file,
        )
        print("Verifying dstack offer matching for the new fleet", flush=True)
        self.dstack.verify_fleet_offer(state.fleet_name)
        print(
            f"Submitting dstack task {state.run_id} to fleet {state.fleet_name}",
            flush=True,
        )
        state.dstack_run_name = state.run_id
        state.status = "submitting"
        self.store.put(state)
        self.dstack.submit(state)
        started_status = self.dstack.wait_until_started(state.run_id)
        state.status = "submitted"
        state.last_error = None
        self.store.put(state)
        print(
            f"dstack run {state.run_id} reached {started_status}; "
            "follow task status in dstack",
            flush=True,
        )

    def reconcile(
        self, state: RunState, actual_instances: dict[str, dict[str, Any]]
    ) -> None:
        print(
            "Reconciling Packet run "
            f"{state.run_id}: state={state.status} attempt={state.attempts} "
            f"instance={state.instance_id or 'none'}",
            flush=True,
        )
        if state.status in {"succeeded", "failed"}:
            if state.instance_id or state.fleet_name:
                self._cleanup(state)
                self.store.put(state)
            return
        if state.status in {"provisioning", "submitting"}:
            updated = datetime.fromisoformat(state.updated_at.replace("Z", "+00:00"))
            if (
                datetime.now(UTC) - updated
            ).total_seconds() < PROVISIONING_LEASE_SECONDS:
                # The submission job still owns this state. Taking it over too
                # early can launch duplicate capacity or submit the task twice.
                return
            if state.status == "provisioning" and not state.instance_id:
                state.last_error = (
                    "provisioning ended before Packet returned an instance"
                )
                self.packet.terminate_named(
                    f"{PACKET_PREFIX}{state.run_id}-{state.attempts}"
                )
                self.launch_attempt(state)
                return
        if state.status == "desired":
            if state.instance_id or state.fleet_name:
                self._cleanup(state)
                self.store.put(state)
            self.launch_attempt(state)
            return
        status, reason = self.dstack.run_status(state.dstack_run_name or state.run_id)
        host_missing = bool(
            state.instance_id and state.instance_id not in actual_instances
        )
        print(
            f"Observed dstack run {state.dstack_run_name or state.run_id}: "
            f"status={status or 'not-found'} reason={reason or 'none'} "
            f"packet_host_missing={host_missing}",
            flush=True,
        )
        if status in TERMINAL_DSTACK_SUCCESS:
            self._cleanup(state)
            state.status = "succeeded"
            self.store.put(state)
        elif status in TERMINAL_DSTACK_FAILURE:
            self._cleanup(state)
            state.last_error = reason or f"dstack status {status}"
            if _retryable_host_failure(status, reason):
                self.launch_attempt(state)
            else:
                state.status = "failed"
                self.store.put(state)
        elif status is not None:
            state.status = "running"
            state.last_error = (
                "Packet host is absent while dstack still reports a nonterminal run"
                if host_missing
                else None
            )
            self.store.put(state)
        elif state.status == "provisioning":
            self._cleanup(state)
            if not state.instance_id:
                self.packet.terminate_named(
                    f"{PACKET_PREFIX}{state.run_id}-{state.attempts}"
                )
            state.last_error = "stale provisioning attempt had no submitted dstack task"
            self.launch_attempt(state)
        else:
            state.last_error = (
                "dstack run status is unavailable; preserving existing state and "
                "refusing to launch replacement capacity"
            )
            self.store.put(state)
            print(f"Reconciliation deferred: {state.last_error}", flush=True)

    def sweep_orphans(
        self, states: list[RunState], instances: list[dict[str, Any]]
    ) -> None:
        owned = {state.instance_id for state in states if state.instance_id}
        now = datetime.now(UTC)
        for instance in instances:
            instance_id = str(instance.get("id", ""))
            if (
                not str(instance.get("name", "")).startswith(PACKET_PREFIX)
                or instance_id in owned
            ):
                continue
            created = instance.get("createdAt") or instance.get("created_at")
            if created:
                age = now - datetime.fromisoformat(str(created).replace("Z", "+00:00"))
                if age.total_seconds() < 3600:
                    continue
            self.packet.terminate(instance_id)


def _identity_file() -> tempfile.NamedTemporaryFile[str]:
    key = os.environ.get("PACKET_SSH_PRIVATE_KEY", "")
    if not key:
        raise ValueError("PACKET_SSH_PRIVATE_KEY is required")
    stream = tempfile.NamedTemporaryFile("w", suffix=".key", encoding="utf-8")
    stream.write(key.rstrip() + "\n")
    stream.flush()
    os.chmod(stream.name, 0o600)
    return stream


def _public_key(identity_file: Path) -> str:
    result = subprocess.run(
        ["ssh-keygen", "-y", "-f", str(identity_file)],
        check=True,
        text=True,
        capture_output=True,
    )
    value = result.stdout.strip()
    if not value:
        raise ValueError("PACKET_SSH_PRIVATE_KEY did not produce a public key")
    return value


def _public_key_fingerprint(identity_file: Path) -> str:
    result = subprocess.run(
        ["ssh-keygen", "-lf", str(identity_file), "-E", "sha256"],
        check=True,
        text=True,
        capture_output=True,
    )
    fields = result.stdout.split()
    if len(fields) < 2 or not fields[1].startswith("SHA256:"):
        raise ValueError("PACKET_SSH_PRIVATE_KEY did not produce a fingerprint")
    return fields[1]


def _dataset_disk_requirement(
    dataset_id: str, *, bucket: str, endpoint_url: str | None
) -> int:
    manifest = download_manifest(
        bucket=bucket,
        dataset_id=dataset_id,
        aws=AwsCli(endpoint_url),
    )
    staging_bytes = required_staging_bytes(manifest)
    required = staging_bytes + PACKET_DISK_RESERVE_BYTES
    print(
        "Packet disk budget: "
        f"dataset staging={staging_bytes / 1024**3:.1f} GiB, "
        f"container/checkpoint reserve={PACKET_DISK_RESERVE_BYTES / 1024**3:.0f} GiB, "
        f"required free={required / 1024**3:.1f} GiB",
        flush=True,
    )
    return required


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("submit", "reconcile"))
    args = parser.parse_args()
    packet = PacketClient(
        os.environ["PACKET_API_KEY"],
        api_url=os.getenv("PACKET_API_URL", "https://dash.packet.ai"),
    )
    bucket = os.environ["S3_BUCKET"]
    endpoint_url = os.getenv("S3_ENDPOINT_URL") or None
    store = S3StateStore(bucket, endpoint_url)
    with _identity_file() as identity:
        identity_file = Path(identity.name)
        fingerprint = _public_key_fingerprint(identity_file)
        public_key = _public_key(identity_file)
        ssh_key_id = packet.resolve_ssh_key(
            public_key,
            fingerprint,
            os.getenv("PACKET_SSH_KEY_ID") or os.getenv("CLOUD_SSH_KEY_ID"),
        )
        print(
            f"Using registered Packet SSH key {ssh_key_id} ({fingerprint})",
            flush=True,
        )
        coordinator = PacketCoordinator(
            packet,
            DstackClient(),
            store,
            identity_file=identity_file,
            public_key=public_key,
            storage_block_id=os.getenv("PACKET_STORAGE_BLOCK_ID") or None,
        )
        if args.command == "submit":
            run_id = os.environ["RUN_ID"]
            if store.get(run_id):
                raise RuntimeError(f"Packet run {run_id} already exists")
            now = _utc_now()
            state = RunState(
                run_id=run_id,
                gpu=os.environ["DSTACK_GPU"],
                config=os.environ["CONFIG_PATH"],
                dataset_id=os.environ["DATASET_ID"],
                mode=os.environ["RUN_MODE"],
                created_at=now,
                updated_at=now,
                source_commit=os.environ.get("SOURCE_COMMIT", ""),
                display_name=os.environ.get("RUN_DISPLAY_NAME", ""),
                batch_candidates=os.environ.get("BATCH_CANDIDATES", "16,32,64,96,128"),
                ssh_key_id=ssh_key_id,
                required_disk_bytes=_dataset_disk_requirement(
                    os.environ["DATASET_ID"],
                    bucket=bucket,
                    endpoint_url=endpoint_url,
                ),
            )
            store.put(state)
            coordinator.submit_with_retries(state)
            print(json.dumps(asdict(state), indent=2, sort_keys=True))
        else:
            states = store.list()
            instances = packet.list_instances()
            actual = {str(item.get("id")): item for item in instances}
            for state in states:
                try:
                    if state.required_disk_bytes == 0 and state.status not in {
                        "succeeded",
                        "failed",
                    }:
                        state.required_disk_bytes = _dataset_disk_requirement(
                            state.dataset_id,
                            bucket=bucket,
                            endpoint_url=endpoint_url,
                        )
                        store.put(state)
                    coordinator.reconcile(state, actual)
                except Exception as error:
                    state.last_error = str(error)
                    store.put(state)
                    print(f"reconcile failed for {state.run_id}: {error}")
            coordinator.sweep_orphans(states, instances)


if __name__ == "__main__":
    main()
