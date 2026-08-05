"""Provider-agnostic GPU instance provisioning and lifecycle CLI orchestrator.

Supports RunPod (default) and Packet.ai cloud providers.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request
from typing import Any


def _make_json_request(
    url: str,
    api_key: str,
    method: str = "GET",
    payload: dict[str, Any] | None = None,
    headers_extra: dict[str, str] | None = None,
) -> dict[str, Any]:
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json",
        "User-Agent": "antigravity-cloud-orchestrator/1.0",
    }
    if headers_extra:
        headers.update(headers_extra)

    data = json.dumps(payload).encode("utf-8") if payload is not None else None
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=30) as response:
            body = response.read().decode("utf-8")
            return json.loads(body) if body else {}
    except urllib.error.HTTPError as err:
        error_body = err.read().decode("utf-8")
        raise RuntimeError(f"HTTP {err.code} from {url}: {error_body}") from err


class RunPodProvider:
    """RunPod cloud provider implementation using GraphQL API."""

    GPU_MAPPING = {
        "rtx4090": "NVIDIA GeForce RTX 4090",
        "a100": "NVIDIA A100-SXM4-80GB",
        "a100-pcie": "NVIDIA A100 PCIe 80GB",
        "l40s": "NVIDIA L40S",
    }

    def __init__(self, api_key: str, api_url: str | None = None) -> None:
        self.api_key = api_key
        self.graphql_url = (api_url or "https://api.runpod.io/graphql").rstrip("/")

    def _graphql_query(self, query: str, variables: dict[str, Any] | None = None) -> dict[str, Any]:
        url = f"{self.graphql_url}?api_key={self.api_key}"
        payload = {"query": query, "variables": variables or {}}
        res = _make_json_request(url, self.api_key, method="POST", payload=payload)
        if "errors" in res and res["errors"]:
            raise RuntimeError(f"RunPod GraphQL Error: {res['errors']}")
        return res.get("data", {})

    def launch(
        self,
        gpu_type: str,
        volume_id: str | None,
        ssh_key_id: str | None,
        name_prefix: str,
        container_image: str,
        timeout_seconds: int = 300,
    ) -> dict[str, Any]:
        mapped_gpu = self.GPU_MAPPING.get(gpu_type.lower(), gpu_type)
        name = f"{name_prefix}{int(time.time())}"

        mutation = """
        mutation PodFindAndDeployOnDemand($input: PodFindAndDeployOnDemandInput!) {
          podFindAndDeployOnDemand(input: $input) {
            id
            name
            desiredStatus
            imageName
          }
        }
        """

        input_data: dict[str, Any] = {
            "name": name,
            "imageName": container_image,
            "gpuTypeId": mapped_gpu,
            "gpuCount": 1,
            "volumeInGb": 50,
            "containerDiskInGb": 50,
            "minVcpuCount": 4,
            "minMemoryInGb": 16,
            "ports": "22/tcp",
        }
        if volume_id:
            input_data["networkVolumeId"] = volume_id

        res = self._graphql_query(mutation, {"input": input_data})
        pod_info = res.get("podFindAndDeployOnDemand")
        if not pod_info or not pod_info.get("id"):
            raise RuntimeError(f"RunPod failed to return pod details: {res}")

        pod_id = pod_info["id"]

        # Poll pod status until active & public IP / SSH port assigned
        start_time = time.perf_counter()
        query_pod = """
        query Pod($podId: String!) {
          pod(input: {podId: $podId}) {
            id
            name
            desiredStatus
            runtime {
              uptimeInSeconds
              ports {
                ip
                isIpPublic
                privatePort
                publicPort
              }
            }
          }
        }
        """

        while time.perf_counter() - start_time < timeout_seconds:
            pod_res = self._graphql_query(query_pod, {"podId": pod_id})
            pod_data = pod_res.get("pod") or {}
            runtime = pod_data.get("runtime") or {}
            ports = runtime.get("ports") or []

            ssh_port_info = next((p for p in ports if p.get("privatePort") == 22), None)
            ip_address = ssh_port_info.get("ip") if ssh_port_info else None
            public_port = ssh_port_info.get("publicPort") if ssh_port_info else 22

            if pod_data.get("desiredStatus") == "RUNNING" and ip_address:
                return {
                    "instance_id": pod_id,
                    "name": name,
                    "provider": "runpod",
                    "gpu_type": gpu_type,
                    "ip_address": ip_address,
                    "ssh_port": public_port,
                    "status": "active",
                    "details": pod_data,
                }
            time.sleep(5)

        raise TimeoutError(f"RunPod instance {pod_id} failed to become active within {timeout_seconds}s")

    def terminate(self, instance_id: str) -> dict[str, Any]:
        mutation = """
        mutation PodTerminate($podId: String!) {
          podTerminate(input: {podId: $podId})
        }
        """
        try:
            res = self._graphql_query(mutation, {"podId": instance_id})
            return {"instance_id": instance_id, "provider": "runpod", "status": "terminated", "details": res}
        except Exception as err:
            return {"instance_id": instance_id, "provider": "runpod", "status": "error", "error": str(err)}


class PacketProvider:
    """Packet.ai cloud provider implementation using REST API."""

    def __init__(self, api_key: str, api_url: str | None = None) -> None:
        self.api_key = api_key
        self.api_url = (api_url or "https://dash.packet.ai").rstrip("/")

    def launch(
        self,
        gpu_type: str,
        volume_id: str | None,
        ssh_key_id: str | None,
        name_prefix: str,
        container_image: str,
        timeout_seconds: int = 300,
    ) -> dict[str, Any]:
        url = f"{self.api_url}/api/v1/instances"
        name = f"{name_prefix}{int(time.time())}"
        payload: dict[str, Any] = {
            "name": name,
            "gpu_type": gpu_type,
        }
        if volume_id:
            payload["existing_shared_volume_id"] = volume_id
        if ssh_key_id:
            payload["ssh_key_id"] = ssh_key_id

        response = _make_json_request(url, self.api_key, method="POST", payload=payload)
        instance_id = response.get("id") or response.get("instance_id")
        if not instance_id:
            raise RuntimeError(f"Packet API did not return an instance ID: {response}")

        start_time = time.perf_counter()
        while time.perf_counter() - start_time < timeout_seconds:
            status_url = f"{url}/{instance_id}"
            details = _make_json_request(status_url, self.api_key, method="GET")
            state = details.get("state") or details.get("status")
            ip_address = details.get("ip_address") or details.get("public_ip")

            if state in {"active", "running"} and ip_address:
                return {
                    "instance_id": instance_id,
                    "name": name,
                    "provider": "packet",
                    "gpu_type": gpu_type,
                    "ip_address": ip_address,
                    "ssh_port": 22,
                    "status": "active",
                    "details": details,
                }
            elif state in {"failed", "error"}:
                raise RuntimeError(f"Instance {instance_id} entered error state: {details}")
            time.sleep(5)

        raise TimeoutError(f"Packet instance {instance_id} failed to become active within {timeout_seconds}s")

    def terminate(self, instance_id: str) -> dict[str, Any]:
        url = f"{self.api_url}/api/v1/instances/{instance_id}"
        try:
            response = _make_json_request(url, self.api_key, method="DELETE")
            return {"instance_id": instance_id, "provider": "packet", "status": "terminated", "details": response}
        except Exception as err:
            return {"instance_id": instance_id, "provider": "packet", "status": "error", "error": str(err)}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Provider-agnostic GPU Cloud Orchestrator CLI")
    subparsers = parser.add_subparsers(dest="command", required=True)

    launch_p = subparsers.add_parser("launch")
    launch_p.add_argument("--provider", default=os.getenv("CLOUD_PROVIDER", "runpod"), choices=["runpod", "packet"])
    launch_p.add_argument("--api-url", default=os.getenv("CLOUD_API_URL"))
    launch_p.add_argument("--api-key", default=os.getenv("CLOUD_API_KEY"))
    launch_p.add_argument("--gpu-type", default="rtx4090")
    launch_p.add_argument("--volume-id", default=os.getenv("CLOUD_PERSISTENT_VOLUME_ID"))
    launch_p.add_argument("--ssh-key-id", default=os.getenv("CLOUD_SSH_KEY_ID"))
    launch_p.add_argument("--name-prefix", default=os.getenv("CLOUD_INSTANCE_PREFIX", "vi-gha-"))
    launch_p.add_argument(
        "--container-image",
        default=os.getenv(
            "CLOUD_CONTAINER_IMAGE", "runpod/pytorch:2.5.1-py3.11-cuda12.4.1-devel-ubuntu22.04"
        ),
    )

    term_p = subparsers.add_parser("terminate")
    term_p.add_argument("--provider", default=os.getenv("CLOUD_PROVIDER", "runpod"), choices=["runpod", "packet"])
    term_p.add_argument("--api-url", default=os.getenv("CLOUD_API_URL"))
    term_p.add_argument("--api-key", default=os.getenv("CLOUD_API_KEY"))
    term_p.add_argument("--instance-id", required=True)

    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.api_key:
        print(json.dumps({"error": "CLOUD_API_KEY is required"}), file=sys.stderr)
        sys.exit(1)


    provider_name = args.provider.lower()
    if provider_name == "runpod":
        provider = RunPodProvider(api_key=args.api_key, api_url=args.api_url)
    elif provider_name == "packet":
        provider = PacketProvider(api_key=args.api_key, api_url=args.api_url)
    else:
        print(json.dumps({"error": f"Unsupported provider: {provider_name}"}), file=sys.stderr)
        sys.exit(1)

    if args.command == "launch":
        try:
            res = provider.launch(
                gpu_type=args.gpu_type,
                volume_id=args.volume_id,
                ssh_key_id=args.ssh_key_id,
                name_prefix=args.name_prefix,
                container_image=args.container_image,
            )
            print(json.dumps(res, indent=2))
        except Exception as err:
            print(json.dumps({"status": "error", "error": str(err)}), file=sys.stderr)
            sys.exit(1)

    elif args.command == "terminate":
        res = provider.terminate(instance_id=args.instance_id)
        print(json.dumps(res, indent=2))
        if res.get("status") == "error":
            sys.exit(1)


if __name__ == "__main__":
    main()
