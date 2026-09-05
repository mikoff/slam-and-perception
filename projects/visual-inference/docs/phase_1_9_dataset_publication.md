# Phase 1.9 immutable dataset publication

Status: archive and manifest published and verified; the owner-approved local
HBB/quad loader gate passed. Clean-worker archive portability is deferred to the
first normal cloud staging.

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

## Local HBB/quad loader gate

On 2026-09-05, commit `89389b7428cd6b1fc4cecbfd31fc4ad46243a0c6`
loaded the production train and validation indexes through both
`IndexedCocoProposalDataset` and `QuadProposalDataset`. Each check required the
prebuilt index, sampled one item from every source, then compared it with the
same item returned by a spawned DataLoader worker. Image tensors and geometry
were finite, masks and image paths were valid, parent/worker tensors matched,
and index size and modification time did not change.

| Split | Reader | Images | Positive | Ignore | Trusted background | Init | Total | Peak RSS |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| train | HBB | 266,977 | 3,765,518 | 45,846 | 21,473,325 | 188.85 s | 210.35 s | 954.1 MiB |
| train | quad | 266,977 | 3,765,518 | 45,846 | 21,473,325 | 169.94 s | 189.19 s | 981.7 MiB |
| val | HBB | 32,268 | 569,716 | 1,906 | 3,041,953 | 24.24 s | 38.37 s | 981.7 MiB |
| val | quad | 32,268 | 569,716 | 1,906 | 3,041,953 | 17.11 s | 31.35 s | 981.7 MiB |

The identical state totals verify HBB/quad supervision parity; retained
geometry can legitimately differ after representation-specific validation. The
machine-readable summary is in `phase_1_9_local_loader_smoke.json`.

The full remote archive is not redundantly downloaded to the publishing host.
At the first normal cloud staging, the worker must download it by dataset ID,
verify its size and SHA-256, safely extract it, reconcile extracted size,
confirm both indexes, and load HBB and quad batches without local-source access.
This deferred check is the archive-portability gate because the local dataset's
image paths resolve through the mounted source disk.
