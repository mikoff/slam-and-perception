"""RunPod cloud provider implementation using GraphQL API."""

from __future__ import annotations

import time
from typing import Any

from ..base import AbstractCloudProvider, make_json_request


class RunPodProvider(AbstractCloudProvider):
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
        res = make_json_request(url, self.api_key, method="POST", payload=payload)
        if "errors" in res and res["errors"]:
            raise RuntimeError(f"RunPod GraphQL Error: {res['errors']}")
        return res.get("data", {})

    def start_instance(
        self,
        gpu_type: str,
        container_image: str,
        name_prefix: str = "vi-gha-",
        volume_id: str | None = None,
        ssh_key_id: str | None = None,
        timeout_seconds: int = 300,
        extra_params: dict[str, Any] | None = None,
        **kwargs: Any,
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
        if volume_id and volume_id.strip().lower() not in {"none", "null", "false", ""}:
            input_data["networkVolumeId"] = volume_id

        res = self._graphql_query(mutation, {"input": input_data})
        pod_info = res.get("podFindAndDeployOnDemand")
        if not pod_info or not pod_info.get("id"):
            raise RuntimeError(f"RunPod failed to return pod details: {res}")

        pod_id = pod_info["id"]

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
                    "ssh_user": "devuser",
                    "status": "active",
                    "details": pod_data,
                }
            time.sleep(5)

        raise TimeoutError(f"RunPod instance {pod_id} failed to become active within {timeout_seconds}s")

    def kill_instance(self, instance_id: str, **kwargs: Any) -> dict[str, Any]:
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
