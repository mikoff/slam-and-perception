"""Cloud execution daemon and monitor for visual-inference training.

Handles robust subprocess execution, real-time log streaming, signal traps,
periodic S3 checkpoint syncs, watchdog pings, and automatic cloud node self-termination.
"""

from __future__ import annotations

import argparse
import atexit
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import NoReturn

S3_BUCKET = os.getenv("S3_BUCKET", "")
S3_ENDPOINT = os.getenv("S3_ENDPOINT_URL", "")
INSTANCE_ID = os.getenv("INSTANCE_ID", "")
CLOUD_PROVIDER = os.getenv("CLOUD_PROVIDER", "packet")
CLOUD_API_KEY = os.getenv("CLOUD_API_KEY", "")


def sync_output_to_s3(output_dir: Path, run_name: str) -> None:
    if not S3_BUCKET or not output_dir.exists():
        return
    cmd = [
        "aws", "s3", "sync",
        str(output_dir),
        f"s3://{S3_BUCKET}/runs/{run_name}",
        "--no-progress", "--quiet"
    ]
    if S3_ENDPOINT:
        cmd.extend(["--endpoint-url", S3_ENDPOINT])
    try:
        subprocess.run(cmd, check=False)
    except Exception as err:
        sys.stderr.write(f"--> [Daemon Warning] S3 sync failed: {err}\n")


def ping_janitor() -> None:
    if not S3_BUCKET or not INSTANCE_ID:
        return
    payload = f'{{"instance_id": "{INSTANCE_ID}", "provider": "{CLOUD_PROVIDER}", "timestamp": {int(time.time())}}}'
    ping_path = Path("/tmp/janitor_ping.json")
    ping_path.write_text(payload)
    cmd = [
        "aws", "s3", "cp",
        str(ping_path),
        f"s3://{S3_BUCKET}/janitor/{INSTANCE_ID}.json",
        "--quiet"
    ]
    if S3_ENDPOINT:
        cmd.extend(["--endpoint-url", S3_ENDPOINT])
    try:
        subprocess.run(cmd, check=False)
    except Exception:
        pass


def self_terminate() -> None:
    if not CLOUD_API_KEY or not INSTANCE_ID:
        return
    sys.stdout.write(f"--> [Daemon] Self-terminating cloud instance {INSTANCE_ID}...\n")
    sys.stdout.flush()
    cmd = [
        sys.executable,
        "scripts/cloud/cli.py",
        "kill_instance",
        "--provider", CLOUD_PROVIDER,
        "--instance-id", INSTANCE_ID,
    ]
    env = dict(os.environ, CLOUD_API_KEY=CLOUD_API_KEY)
    try:
        subprocess.run(cmd, env=env, check=False)
    except Exception as err:
        sys.stderr.write(f"--> [Daemon Error] Failed to self-terminate instance: {err}\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Cloud training execution daemon")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--run-name", type=str, required=True)
    parser.add_argument("training_cmd", nargs=argparse.REMAINDER)
    args = parser.parse_args()

    if not args.training_cmd:
        sys.stderr.write("Error: No training command provided to run_cloud_daemon.py\n")
        sys.exit(1)

    log_file = Path("/tmp/training.log")
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"=== [Cloud Training Daemon Started] ===")
    print(f"--> Target Output Dir: {output_dir}")
    print(f"--> Executing Command: {' '.join(args.training_cmd)}")
    sys.stdout.flush()

    process = subprocess.Popen(
        args.training_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    def shutdown_handler(signum: int, frame: object) -> None:
        sys.stdout.write(f"\n--> [Daemon] Signal {signum} received! Cleaning up...\n")
        sys.stdout.flush()
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
        sync_output_to_s3(output_dir, args.run_name)
        self_terminate()
        sys.exit(1)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    last_sync_time = time.time()

    with log_file.open("a", encoding="utf-8") as log_stream:
        while True:
            line = process.stdout.readline() if process.stdout else ""
            if not line and process.poll() is not None:
                break
            if line:
                sys.stdout.write(line)
                sys.stdout.flush()
                log_stream.write(line)
                log_stream.flush()

            now = time.time()
            if now - last_sync_time >= 300: # Every 5 minutes
                sync_output_to_s3(output_dir, args.run_name)
                ping_janitor()
                last_sync_time = now

    exit_code = process.poll() or 0
    sys.stdout.write(f"\n=== [Training Process Finished with exit code {exit_code}] ===\n")
    sys.stdout.flush()

    # Final sync & cleanup
    sync_output_to_s3(output_dir, args.run_name)
    if exit_code == 0:
        self_terminate()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
