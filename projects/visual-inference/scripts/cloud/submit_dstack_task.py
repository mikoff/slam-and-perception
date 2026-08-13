"""Submit the rendered native dstack task from the validated cloud environment."""

from __future__ import annotations

import os

from packet_bridge import DstackClient, RunState, _utc_now


def main() -> None:
    now = _utc_now()
    state = RunState(
        run_id=os.environ["RUN_ID"],
        gpu=os.environ["DSTACK_GPU"],
        config=os.environ["CONFIG_PATH"],
        dataset_id=os.environ["DATASET_ID"],
        mode=os.environ["RUN_MODE"],
        created_at=now,
        updated_at=now,
        source_commit=os.environ["SOURCE_COMMIT"],
        display_name=os.getenv("RUN_DISPLAY_NAME", ""),
        batch_candidates=os.getenv("BATCH_CANDIDATES", "16,32,64,96,128"),
    )
    DstackClient().submit(state)


if __name__ == "__main__":
    main()
