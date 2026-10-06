# Proposal Object Contract

Status: **approved** by `mikoff`.

## Definition

A positive is a bounded, visually identifiable physical instance whose crop
could be usefully classified or described by the downstream SigLIP stage.
Stuff, surfaces, textures, reflections, shadows, ego-platform bodywork, and
scene-layout regions are background. Regions that might contain several
unresolved instances or whose semantics are uncertain are ignored rather than
forced positive or negative.

## Supervision states

- **Positive:** preserve the spatial annotation as a proposal target.
- **Trusted negative:** preserve only the annotated pixels as explicit
  background; incomplete coverage does not authorize negatives elsewhere.
- **Ignore:** preserve the spatial region and exclude it from positive and
  negative loss.
- **Drop:** remove the annotation with a declared reason; the surrounding area
  remains unlabeled rather than becoming trusted background.

## Global rules

- Ignore a contained component when its retained parent already represents the
  useful crop. License plates are ignored unless plate proposals become an
  explicit downstream requirement.
- Preserve crowd and group regions as ignore unless recoverable source identity
  provides clean individual instances.
- Void, soiling, transparency, and other uncertain masks are ignore regions.
- Valid physical instances take precedence over ignore; ignore takes precedence
  over trusted negative.

## Approved evidence

The authoritative state matrix and review rationale are stored in:

- `automotive_taxonomy_mapping.json` under `proposal_object_contract`;
- `dataset-pipeline/configs/proposal_object_contract_review.csv`.

The review resolves all 163 declared source categories: 139 positive, 8 trusted
negative, 8 ignore, and 8 drop. It includes explicit source-quality exceptions
for partial drivable-area coverage, unreliable WoodScape labels, and categories
with fewer than 100 examples.

This document freezes the product decision. Phase 1.3 will make the approved
state a generated-manifest property shared by HBB and quadrilateral loaders;
until that migration, legacy converter behavior is not evidence of policy.
