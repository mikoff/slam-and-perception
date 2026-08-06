"""Packet.ai cloud provider implementation using REST API."""

from __future__ import annotations

import os
import socket
import time
from typing import Any

from ..base import AbstractCloudProvider, make_json_request


class PacketProvider(AbstractCloudProvider):
    """Packet.ai cloud provider implementation using REST API."""

    def __init__(self, api_key: str, api_url: str | None = None) -> None:
        self.api_key = api_key
        self.api_url = (api_url or "https://dash.packet.ai").rstrip("/")

    def _resolve_pool_id(
        self, gpu_type: str, extra_params: dict[str, Any] | None, kwargs: dict[str, Any]
    ) -> str:
        # 1. Explicit pool_id passed via extra_params, kwargs, or env vars
        if extra_params and extra_params.get("pool_id"):
            return str(extra_params["pool_id"])
        if kwargs.get("pool_id"):
            return str(kwargs["pool_id"])
        env_pool = os.getenv("CLOUD_POOL_ID") or os.getenv("PACKET_POOL_ID") or os.getenv("POOL_ID")
        if env_pool:
            return env_pool

        # 2. Dynamic API resolution from https://dash.packet.ai/api/v1/launch-options
        pools: list[dict[str, Any]] = []
        try:
            url = f"{self.api_url}/api/v1/launch-options"
            resp = make_json_request(url, self.api_key, method="GET")
            pools = resp.get("data", {}).get("pools", [])

            clean_gpu = gpu_type.lower().replace("-", "").replace("_", "").replace(" ", "")
            matching_pools: list[dict[str, Any]] = []
            for p in pools:
                model = p.get("gpu_model", "").lower().replace("-", "").replace("_", "").replace(" ", "")
                p_name = p.get("name", "").lower().replace("-", "").replace("_", "").replace(" ", "")
                if clean_gpu in model or clean_gpu in p_name:
                    matching_pools.append(p)
                elif clean_gpu in {"rtx6000", "rtx6000pro", "6000", "rtxpro6000"} and ("6000" in model or "6000" in p_name):
                    matching_pools.append(p)
                elif clean_gpu in {"rtx4090", "4090"} and ("4090" in model or "4090" in p_name):
                    matching_pools.append(p)

            # If rtx4090 requested but no 4090 pool exists on Packet.ai, fall back to budget GPU (rtx6000)
            if not matching_pools and clean_gpu in {"rtx4090", "4090"}:
                for p in pools:
                    model = p.get("gpu_model", "").lower().replace("-", "").replace("_", "").replace(" ", "")
                    p_name = p.get("name", "").lower().replace("-", "").replace("_", "").replace(" ", "")
                    if "6000" in model or "6000" in p_name:
                        matching_pools.append(p)

            # Prefer pools with available GPUs (> 0)
            available_pools = [p for p in matching_pools if p.get("available_gpus", 0) > 0]
            chosen = available_pools[0] if available_pools else (matching_pools[0] if matching_pools else None)
            if chosen and chosen.get("id"):
                return str(chosen["id"])
        except ValueError:
            raise
        except Exception:
            pass

        # If pool resolution failed or no matching pool found for gpu_type:
        available_info = [
            f"ID {p.get('id')}: {p.get('gpu_model')} ({p.get('name')}, {p.get('available_gpus', 0)} avail)"
            for p in pools
        ]
        pool_summary = "\n  ".join(available_info) if available_info else "No pools returned by Packet API"
        raise ValueError(
            f"Could not resolve Packet.ai pool_id for GPU type '{gpu_type}'.\n"
            f"Available pools on Packet.ai:\n  {pool_summary}\n"
            f"Please specify a valid --gpu-type (e.g. 'rtx6000', 'l40s', 'a100', 'b200') or pass pool_id in extra_params."
        )

    def _resolve_ssh_key_id(
        self, ssh_key_id: str | None, extra_params: dict[str, Any] | None, kwargs: dict[str, Any]
    ) -> str | None:
        if ssh_key_id and ssh_key_id.strip().lower() not in {"none", "null", "false", ""}:
            return ssh_key_id
        if extra_params and extra_params.get("ssh_key_id"):
            return str(extra_params["ssh_key_id"])
        env_key = os.getenv("CLOUD_SSH_KEY_ID") or os.getenv("PACKET_SSH_KEY_ID")
        if env_key:
            return env_key

        # Dynamic auto-detection of registered SSH keys on Packet.ai
        try:
            url = f"{self.api_url}/api/v1/ssh-keys"
            resp = make_json_request(url, self.api_key, method="GET")
            keys = resp.get("data", [])
            if isinstance(keys, list) and keys and keys[0].get("id"):
                return str(keys[0]["id"])
        except Exception:
            pass

        return None

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
        url = f"{self.api_url}/api/v1/instances"
        name = f"{name_prefix}{int(time.time())}"
        pool_id = self._resolve_pool_id(gpu_type, extra_params, kwargs)
        resolved_ssh_key_id = self._resolve_ssh_key_id(ssh_key_id, extra_params, kwargs)

        payload: dict[str, Any] = {
            "name": name,
            "gpu_type": gpu_type,
            "pool_id": pool_id,
        }

        # Shared volumes are not supported for RTX 4090 instances on Packet.ai
        if volume_id and volume_id.strip().lower() not in {"none", "null", "false", ""}:
            if gpu_type.lower() != "rtx4090":
                payload["existing_shared_volume_id"] = volume_id
        if resolved_ssh_key_id:
            payload["ssh_key_id"] = resolved_ssh_key_id
            payload["ssh_key_ids"] = [resolved_ssh_key_id]

        response = make_json_request(url, self.api_key, method="POST", payload=payload)
        resp_data = response.get("data") if isinstance(response.get("data"), dict) else response
        instance_id = (
            resp_data.get("instance_id")
            or resp_data.get("id")
            or response.get("id")
            or response.get("instance_id")
        )
        if not instance_id:
            raise RuntimeError(f"Packet API did not return an instance ID: {response}")

        start_time = time.perf_counter()
        while time.perf_counter() - start_time < timeout_seconds:
            status_url = f"{url}/{instance_id}"
            details = make_json_request(status_url, self.api_key, method="GET")
            details_data = details.get("data") if isinstance(details.get("data"), dict) else details
            instance_obj = (
                details_data.get("instance")
                if isinstance(details_data.get("instance"), dict)
                else details_data
            )
            conn_info = (
                details_data.get("connectionInfo")
                if isinstance(details_data.get("connectionInfo"), dict)
                else {}
            )

            state = (
                instance_obj.get("deploy_status")
                or instance_obj.get("state")
                or instance_obj.get("status")
                or details_data.get("status")
                or ""
            ).lower()

            raw_ip = (
                conn_info.get("ip")
                or instance_obj.get("ip_address")
                or instance_obj.get("public_ip")
                or instance_obj.get("ip")
            )
            if isinstance(raw_ip, list):
                ip_address = raw_ip[0] if raw_ip and raw_ip[0] else None
            else:
                ip_address = raw_ip

            ssh_port = conn_info.get("port") or instance_obj.get("ssh_port") or 22

            if state in {"active", "running"} and ip_address:
                port_num = int(ssh_port)
                tcp_ready = False
                for _ in range(6):
                    try:
                        with socket.create_connection((ip_address, port_num), timeout=3):
                            tcp_ready = True
                            break
                    except Exception:
                        time.sleep(2)

                if tcp_ready or (time.perf_counter() - start_time >= timeout_seconds - 10):
                    return {
                        "instance_id": instance_id,
                        "name": name,
                        "provider": "packet",
                        "gpu_type": gpu_type,
                        "ip_address": ip_address,
                        "ssh_port": ssh_port,
                        "ssh_user": conn_info.get("username") or "ubuntu",
                        "status": "active",
                        "details": details,
                    }
            elif state in {"failed", "error"}:
                raise RuntimeError(f"Instance {instance_id} entered error state: {details}")
            time.sleep(5)

        raise TimeoutError(f"Packet instance {instance_id} failed to become active within {timeout_seconds}s")

    def kill_instance(self, instance_id: str, **kwargs: Any) -> dict[str, Any]:
        url = f"{self.api_url}/api/v1/instances/{instance_id}"
        try:
            response = make_json_request(url, self.api_key, method="DELETE")
            return {"instance_id": instance_id, "provider": "packet", "status": "terminated", "details": response}
        except Exception as err:
            return {"instance_id": instance_id, "provider": "packet", "status": "error", "error": str(err)}
