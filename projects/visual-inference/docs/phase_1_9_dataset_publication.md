# Phase 1.9 immutable dataset publication

Status: archive and manifest published and verified; clean-worker staging and
HBB/quad loader smoke remain pending.

Dataset ID: `phase3-production-bg-policy-v2-2026-08-29`

## Published artifacts

| Artifact | Size | SHA-256 |
|---|---:|---|
| `dataset.tar.gz` | 62,435,399,007 bytes | `9a0671a72cebe7d102d6c147408b159347c9fd8f5e315a11017daca511de1ec8` |
| `dataset-manifest.json` | 541 bytes | `c8e3cb1f4dacbfb5ffa5b6599339404b29fe39a6434cd7320a1ed859e581e92c` |

The native tar/pigz/AWS pipeline streamed 93,304,191,707 extracted bytes without
creating a local archive. The archive upload and streaming SHA process both
returned zero. S3 reported a completed 3,722-part object and no remaining
multipart uploads.

The remote manifest carries its SHA-256 as object metadata and was downloaded
to stdout for a byte-for-byte comparison with the local copy; `cmp` returned
zero. It uses `visual-inference-dataset.v2`, binding the archive hash, archive
and extracted sizes, required indexes, and Phase 1 dataset-contract SHA-256
`0c41b8af31825feb6929067a4cb7128c36569af9c19cb6d14f5cf77dc3ec797a`.

The full remote archive is not redundantly downloaded to the publishing host.
The first clean worker must download it by dataset ID, verify its size and
SHA-256, safely extract it, reconcile extracted size, confirm both indexes, load
HBB and quad samples, and complete a short batch without local-source access.
