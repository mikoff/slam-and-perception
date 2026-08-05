"""Base interface and HTTP utilities for cloud GPU orchestrator providers."""

from __future__ import annotations

import abc
import json
import urllib.error
import urllib.request
from typing import Any


def make_json_request(
    url: str,
    api_key: str,
    method: str = "GET",
    payload: dict[str, Any] | None = None,
    headers_extra: dict[str, str] | None = None,
) -> dict[str, Any]:
    """Execute HTTP JSON request with bearer token authorization."""
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json",
        "User-Agent": "antigravity-cloud-orchestrator/2.0",
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


class AbstractCloudProvider(abc.ABC):
    """Abstract polymorphic base class for cloud GPU providers."""

    @abc.abstractmethod
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
        """Provision and launch a cloud GPU instance."""

    @abc.abstractmethod
    def kill_instance(self, instance_id: str, **kwargs: Any) -> dict[str, Any]:
        """Terminate a cloud GPU instance."""

    # Backward-compatible aliases
    def launch(self, *args: Any, **kwargs: Any) -> dict[str, Any]:
        return self.start_instance(*args, **kwargs)

    def terminate(self, instance_id: str, **kwargs: Any) -> dict[str, Any]:
        return self.kill_instance(instance_id, **kwargs)
