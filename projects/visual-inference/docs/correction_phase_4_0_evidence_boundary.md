# Correction Phase 4.0 evidence boundary

Status: complete; owner confirmed and subsequent Phase 4.1–4.2a gates passed.

## Source boundary

| Item | Identity |
|---|---|
| Phase 2/3 evidence commit | `6c405b7e5f6e5bb87765ce31e123032b37b07d75` |
| Shared-evaluator refactor commit | `6c83b970953e594678b4aa6364f2154c17e83fab` |
| `uv.lock` SHA-256 | `327a20f27cc0bfe1ef3add793791cef735d6c2133627fab5f8a5ce20545bb316` |
| Entry artifact manifest | `configs/benchmarks/correction_phase4_entry_artifacts_v1.json` |
| Manifest SHA-256 | `2436a75331adc7e952125584ec3abb044eb0576da54b640fc73230ea57b814a4` |
| Resolved entry gate SHA-256 | `b1d1dd49b4b44aebc0408d64c6ca384090eebfdf5739a349f6d16b3678a473dc` |

The evidence commits contain the corrected source, tests, contracts, reports,
and the behavior-preserving evaluator refactor. Phase 4 training must bind its
own later clean launch commit; it must not reuse the dirty Phase 3 run identity.

## Published artifacts

The immutable prefix is
`s3://visual-inference/artifacts/proposal-correction/phase3-entry-v1/`.
It contains nine artifacts totaling 230,376,768 bytes:

- the selected step-200 HBB checkpoint and its training contract;
- the retained quad-control checkpoint;
- the frozen HBB baseline utility and suppression-calibration reports;
- both rejected-candidate selection decisions; and
- the step-100 and step-200 gradient diagnostic reports.

The manifest records every local path, durable S3 key, byte size, SHA-256, role,
dataset identity, and evidence commit. It was uploaded last as
`artifact-manifest.json`; the remote object is 4,505 bytes and carries SHA-256
`2436a75331adc7e952125584ec3abb044eb0576da54b640fc73230ea57b814a4`.

## Verification

| Check | Result |
|---|---|
| Local artifact size and SHA-256 | pass, all nine |
| Dataset archive remote size | pass, 62,435,399,007 bytes |
| Dataset manifest remote size/SHA-256 | pass, 541 bytes / `c8e3cb1f…81e92c` |
| Evidence artifact remote size/SHA-256 metadata | pass, all nine |
| Published evidence bytes including manifest | 230,381,273 |
| Incomplete multipart uploads under evidence prefix | none |
| Remote manifest byte comparison | pass |

Large checkpoint payloads were not downloaded again. Verification used remote
object metadata; only the 4,505-byte manifest was downloaded for an exact byte
comparison. The already-published production dataset was not uploaded again.

## Gate decision

Phase 4.0 satisfies the pending commit and durable-artifact requirements in the
conditional Phase 3 entry gate. Phase 4.1 may freeze a new full-run contract
after owner confirmation. Cloud pilot and full retraining still require their
own separate approvals.
