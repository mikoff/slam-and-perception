"""CLI entrypoint for cloud GPU orchestrator."""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any

# Ensure parent scripts directory is in sys.path when executed directly
scripts_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from cloud.base import AbstractCloudProvider
from cloud.providers import PacketProvider, RunPodProvider


def get_provider(provider_name: str, api_key: str, api_url: str | None = None) -> AbstractCloudProvider:
    prov = provider_name.lower()
    if prov == "runpod":
        return RunPodProvider(api_key=api_key, api_url=api_url)
    elif prov == "packet":
        return PacketProvider(api_key=api_key, api_url=api_url)
    else:
        raise ValueError(f"Unsupported cloud provider: {provider_name}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Provider-agnostic GPU Cloud Orchestrator CLI")
    subparsers = parser.add_subparsers(dest="command", required=True)

    def add_common_launch_args(p: argparse.ArgumentParser) -> None:
        p.add_argument("--provider", default=os.getenv("CLOUD_PROVIDER", "packet"), choices=["packet", "runpod"])
        p.add_argument("--api-url", default=os.getenv("CLOUD_API_URL"))
        p.add_argument(
            "--api-key",
            default=os.getenv("CLOUD_API_KEY")
            or os.getenv("RUNPOD_API_KEY")
            or os.getenv("PACKET_API_KEY")
            or os.getenv("RUNPOD_API_TOKEN"),
        )
        p.add_argument("--gpu-type", default=os.getenv("CLOUD_GPU_TYPE", "rtx4090"))
        p.add_argument("--volume-id", default=os.getenv("CLOUD_PERSISTENT_VOLUME_ID"))
        p.add_argument("--ssh-key-id", default=os.getenv("CLOUD_SSH_KEY_ID"))
        p.add_argument("--name-prefix", default=os.getenv("CLOUD_INSTANCE_PREFIX", "vi-gha-"))
        p.add_argument(
            "--container-image",
            default=os.getenv(
                "CLOUD_CONTAINER_IMAGE", "runpod/pytorch:2.5.1-py3.11-cuda12.4.1-devel-ubuntu22.04"
            ),
        )
        p.add_argument(
            "--pool-id",
            default=os.getenv("CLOUD_POOL_ID") or os.getenv("PACKET_POOL_ID") or os.getenv("POOL_ID"),
        )
        p.add_argument("--extra-json", help="JSON string of additional provider parameters")
        p.add_argument("--extra", action="append", help="Extra key=value pair (can be specified multiple times)")

    def add_common_kill_args(p: argparse.ArgumentParser) -> None:
        p.add_argument("--provider", default=os.getenv("CLOUD_PROVIDER", "packet"), choices=["packet", "runpod"])
        p.add_argument("--api-url", default=os.getenv("CLOUD_API_URL"))
        p.add_argument(
            "--api-key",
            default=os.getenv("CLOUD_API_KEY")
            or os.getenv("RUNPOD_API_KEY")
            or os.getenv("PACKET_API_KEY")
            or os.getenv("RUNPOD_API_TOKEN"),
        )
        p.add_argument("--instance-id", required=True)

    start_p = subparsers.add_parser("start_instance")
    add_common_launch_args(start_p)
    launch_p = subparsers.add_parser("launch")
    add_common_launch_args(launch_p)

    kill_p = subparsers.add_parser("kill_instance")
    add_common_kill_args(kill_p)
    term_p = subparsers.add_parser("terminate")
    add_common_kill_args(term_p)

    return parser.parse_args()


def parse_extra_params(args: argparse.Namespace) -> dict[str, Any]:
    extra_params: dict[str, Any] = {}
    if getattr(args, "extra_json", None):
        try:
            extra_params.update(json.loads(args.extra_json))
        except Exception as err:
            raise ValueError(f"Invalid JSON passed to --extra-json: {err}") from err
    if getattr(args, "extra", None):
        for item in args.extra:
            if "=" in item:
                k, v = item.split("=", 1)
                extra_params[k.strip()] = v.strip()
    if getattr(args, "pool_id", None):
        extra_params["pool_id"] = args.pool_id
    return extra_params


def main() -> None:
    args = parse_args()
    if not args.api_key:
        print(json.dumps({"error": "CLOUD_API_KEY is required"}), file=sys.stderr)
        sys.exit(1)

    try:
        provider = get_provider(args.provider, api_key=args.api_key, api_url=args.api_url)
    except Exception as err:
        print(json.dumps({"error": str(err)}), file=sys.stderr)
        sys.exit(1)

    if args.command in {"start_instance", "launch"}:
        try:
            extra_params = parse_extra_params(args)
            res = provider.start_instance(
                gpu_type=args.gpu_type,
                container_image=args.container_image,
                name_prefix=args.name_prefix,
                volume_id=args.volume_id,
                ssh_key_id=args.ssh_key_id,
                extra_params=extra_params,
            )
            print(json.dumps(res, indent=2))
        except Exception as err:
            print(json.dumps({"status": "error", "error": str(err)}), file=sys.stderr)
            sys.exit(1)

    elif args.command in {"kill_instance", "terminate"}:
        res = provider.kill_instance(instance_id=args.instance_id)
        print(json.dumps(res, indent=2))
        if res.get("status") == "error":
            sys.exit(1)


if __name__ == "__main__":
    main()
