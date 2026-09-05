# Automotive dataset pipeline

This project converts pre-downloaded Dataset Ninja Supervisely projects for
nuImages, WoodScape RGB Fisheye, BDD100K Images 100K, and COCO 2017 into
deterministic unified COCO train/validation files. Images remain physically in
the raw extraction and all intermediate and final image trees use links.

## Install

Python 3.11 or newer and `uv` are required. No command downloads dataset data.

```bash
cd projects/visual-inference/dataset-pipeline
uv sync --frozen
```

`uv sync` creates `.venv` in this project. To create/update the lock file after
an intentional dependency change, run `uv lock`.

## Configure

Edit `configs/datasets.yaml`. `workspace_root` must be writable and every
`archive` must point to an existing `.tar`, `.tar.gz`, `.tgz`, or `.tar.zst`.
Relative archive and extraction paths resolve against `workspace_root`.
If discovery finds multiple projects in an extraction, set that dataset's
`project_subpath`. The supplied taxonomy is
`configs/automotive_taxonomy_mapping.json`; unmapped categories are fatal.

Before the first run, verify the exact archive filenames under
`/media/sashamikoff/fb.com-mikoff/datasets/public` and adjust the example paths
if the downloaded files have different names.

The supplied configuration resolves its workspace to
`~/git/skillup/data/visual-inference-datasets`. The archives remain at their configured
absolute paths and are never copied or deleted.

COCO normalization uses the official 2017 instance annotation identities rather
than the Dataset Ninja geometry representations. Fetch and verify the pinned
annotation archive from the repository root before running COCO filtering:

```bash
mkdir -p data/visual-inference-datasets/raw/coco_2017/official
curl -L https://images.cocodataset.org/annotations/annotations_trainval2017.zip \
  -o data/visual-inference-datasets/raw/coco_2017/official/annotations_trainval2017.zip
echo "113a836d90195ee1f884e704da6304dfaaecff1f023f49b6ca93c4aaae470268  data/visual-inference-datasets/raw/coco_2017/official/annotations_trainval2017.zip" \
  | sha256sum --check -
unzip -q data/visual-inference-datasets/raw/coco_2017/official/annotations_trainval2017.zip \
  -d data/visual-inference-datasets/raw/coco_2017/official
```

The extracted train/validation JSON hashes are pinned in `configs/datasets.yaml`
and checked before normalization. A mismatched or missing file is fatal.

## Run

Inspect without extraction:

```bash
uv run python -m dataset_pipeline inspect-archives --config configs/datasets.yaml
```

Run a storage-safe smoke test of at most 100 images per source:

```bash
uv run python -m dataset_pipeline all \
  --config configs/datasets.yaml \
  --limit-images 100
```

Run all images by omitting `--limit-images`. The individual idempotent commands
are `inspect-archives`, `extract`, `discover`, `inspect-projects`, `filter`,
`convert-detection`, `export-coco`, `merge`, `validate`, `preview`,
`verify-links`, `disk-usage`, and `cleanup`. Use `cleanup --dry-run` to inspect
the cleanup plan. `--force-extract` is required to replace an extraction whose
archive fingerprint changed; `--force` replaces other incomplete/incorrect
derived outputs.

The default `all` path runs:

```text
inspect-archives → extract → discover → filter → convert-detection
→ export-coco → merge → validate → preview → cleanup
```

`inspect-projects`, `disk-usage`, and `verify-links` remain available as
standalone diagnostics but are not part of the default run. `--dry-run` is
supported only by `filter` and `cleanup`.

Generate the Phase 1.1 object-contract evidence bundle without changing
conversion behavior:

```bash
uv run dataset-pipeline audit-object-contract \
  --config configs/datasets.yaml \
  --contract-audit-count 100
```

The bundle is written to
`<workspace_root>/reports/proposal_object_contract_audit_bundle`. Review its
`REVIEW_INSTRUCTIONS.md` and export category decisions from `index.html` before
promoting any candidate supervision state into conversion policy.

If the browser cannot save the CSV, copy the rendered page text and recover it:

```bash
uv run dataset-pipeline audit-object-contract-recover-review \
  --config configs/datasets.yaml \
  --contract-review-text /path/to/pasted-review.txt
```

Generate the Phase 1.2 official-identity review after COCO filtering:

```bash
uv run dataset-pipeline audit-source-identity \
  --config configs/datasets.yaml \
  --identity-audit-count 100
```

The bundle at `<workspace_root>/reports/coco_identity_audit_bundle` contains a
deterministic random sample plus every exact-geometry group with distinct
official annotation IDs. Machine-readable source, conversion, removal, and
per-category tier accounting is written to `source_provenance.json`,
`conversion_provenance.json`, `annotation_removals.csv`, and
`annotation_provenance.json`.

If its browser export is unavailable, recover the copied review-page text with:

```bash
uv run dataset-pipeline audit-source-identity-recover-review \
  --config configs/datasets.yaml \
  --identity-review-text /path/to/pasted-review.txt
```

Generated supervision is authoritative in `proposal-manifest.v2`. Category and
contained-component decisions are applied once during manifest generation;
both HBB and quad readers consume the resulting positive, ignore, and trusted
background states. Trusted regions use `positive > ignore > trusted background
> weak` precedence and are reported per source category by count and area.

WoodScape validation is held out from labeled training RGB sequences using the
source timestamps. The raw test split is qualitative only. Generate the owner
review bundle for ego, construction, and grouped-region states after filtering:

```bash
uv run dataset-pipeline audit-woodscape-supervision \
  --config configs/datasets.yaml \
  --supervision-audit-count 24
```

Spatial repairs are written to `spatial_repairs.csv`; unrepairable localized
geometry is recorded in `invalid_geometries.csv` and becomes ignore, never a
positive or trusted negative.

After changing the taxonomy, rebuild every derived stage while reusing the raw
extraction:

```bash
uv run python -m dataset_pipeline all \
  --config configs/datasets.yaml \
  --force \
  --workers 2 \
  --log-level INFO
```

Do not add `--force-extract` for a taxonomy-only change. Generated annotation
and COCO files use compact deterministic JSON; reports remain pretty-printed.

Results are written below `<workspace_root>/output`, with reports under
`<workspace_root>/reports`. Final COCO image paths are relative to `output/`
and resolve through symlinks directly into `raw/`.

Filtering and detection conversion use two bounded worker processes by default.
Use `--workers 1` for sequential execution or increase the value after measuring
the workspace disk and CPU utilization. Archive inspection and extraction remain
sequential to avoid competing reads from the same archive disk.

An `all` run reuses archive inspection results when archive path, size, and
modification time are unchanged. Use `--verify-archives` to force fresh SHA-256
checks and TAR inspection. Progress is reported periodically for long archive
and annotation passes, and every stage reports its elapsed time.

## Tests

```bash
uv run pytest
```

The tests use only synthetic projects and archives.
