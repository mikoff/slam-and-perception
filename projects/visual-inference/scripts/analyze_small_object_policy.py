"""Stream model-coordinate retention evidence from a proposal SQLite index."""

from __future__ import annotations

import argparse
import bisect
import csv
import html
import json
import math
import re
import sqlite3
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

from student_detector.config import Phase3Config, load_phase3_config


SOURCE_SHORT_BINS = (0, 4, 8, 16, 24, 32, 48, 64, 96, 128, 192, 256, math.inf)
MODEL_SHORT_BINS = (0, 1, 2, 4, 8, 12, 16, 24, 32, 48, 64, 96, 128, math.inf)
AREA_BINS = (0, 16, 64, 100, 256, 1_024, 4_096, 16_384, 65_536, math.inf)
ASPECT_BINS = (1, 1.5, 2, 3, 5, 10, 20, math.inf)
HISTOGRAM_BINS = {
    "source_short_side_px": SOURCE_SHORT_BINS,
    "model_short_side_px": MODEL_SHORT_BINS,
    "source_area_px2": AREA_BINS,
    "aspect_ratio": ASPECT_BINS,
}


@dataclass
class Bucket:
    objects: int = 0
    hbb_retained: int = 0
    quad_retained: int = 0
    p3_opportunity: int = 0
    any_grid_opportunity: int = 0
    quad_retained_p3: int = 0
    quad_retained_any_grid: int = 0
    p2_opportunity: int = 0
    quad_retained_p2: int = 0
    p2_min4_retained: int = 0
    p2_min4_retained_grid: int = 0
    quad_retained_512: int = 0
    quad_retained_p3_512: int = 0
    quad_retained_640: int = 0
    quad_retained_p3_640: int = 0

    def add(
        self,
        *,
        hbb: bool,
        quad: bool,
        p3: bool,
        any_grid: bool,
        p2: bool,
        p2_min4: bool,
        quad_512: bool,
        p3_512: bool,
        quad_640: bool,
        p3_640: bool,
    ) -> None:
        self.objects += 1
        self.hbb_retained += hbb
        self.quad_retained += quad
        self.p3_opportunity += p3
        self.any_grid_opportunity += any_grid
        self.quad_retained_p3 += quad and p3
        self.quad_retained_any_grid += quad and any_grid
        self.p2_opportunity += p2
        self.quad_retained_p2 += quad and p2
        self.p2_min4_retained += p2_min4
        self.p2_min4_retained_grid += p2_min4 and p2
        self.quad_retained_512 += quad_512
        self.quad_retained_p3_512 += quad_512 and p3_512
        self.quad_retained_640 += quad_640
        self.quad_retained_p3_640 += quad_640 and p3_640


@dataclass
class Aggregate:
    totals: Bucket = field(default_factory=Bucket)
    histograms: dict[str, list[Bucket]] = field(
        default_factory=lambda: {
            name: [Bucket() for _ in range(len(edges) - 1)]
            for name, edges in HISTOGRAM_BINS.items()
        }
    )
    fpn_primary: Counter[str] = field(default_factory=Counter)
    fpn_eligible: Counter[str] = field(default_factory=Counter)
    distance_bands: Counter[str] = field(default_factory=Counter)

    def add(
        self,
        values: dict[str, float],
        *,
        hbb: bool,
        quad: bool,
        opportunities: dict[str, bool],
        scenarios: dict[str, bool],
        primary_level: str,
        eligible_levels: str,
        distance_band: str,
    ) -> None:
        flags = {
            "hbb": hbb,
            "quad": quad,
            "p3": opportunities["P3"],
            "any_grid": any(opportunities.values()),
            "p2": scenarios["p2"],
            "p2_min4": scenarios["p2_min4"],
            "quad_512": scenarios["quad_512"],
            "p3_512": scenarios["p3_512"],
            "quad_640": scenarios["quad_640"],
            "p3_640": scenarios["p3_640"],
        }
        self.totals.add(**flags)
        for metric, value in values.items():
            edges = HISTOGRAM_BINS[metric]
            index = min(bisect.bisect_right(edges, value) - 1, len(edges) - 2)
            self.histograms[metric][max(index, 0)].add(**flags)
        self.fpn_primary[primary_level] += 1
        self.fpn_eligible[eligible_levels] += 1
        self.distance_bands[distance_band] += 1


def _quad_area(quad: list[list[float]]) -> float:
    return abs(
        sum(
            quad[index][0] * quad[(index + 1) % 4][1]
            - quad[index][1] * quad[(index + 1) % 4][0]
            for index in range(4)
        )
        / 2.0
    )


def _edge_lengths(quad: list[list[float]]) -> list[float]:
    return [math.dist(quad[index], quad[(index + 1) % 4]) for index in range(4)]


def _inside_convex(x: float, y: float, quad: list[list[float]]) -> bool:
    signs = []
    for index in range(4):
        x1, y1 = quad[index]
        x2, y2 = quad[(index + 1) % 4]
        signs.append((x2 - x1) * (y - y1) - (y2 - y1) * (x - x1))
    return all(value >= -1e-6 for value in signs) or all(
        value <= 1e-6 for value in signs
    )


def _grid_opportunity(quad: list[list[float]], stride: int, input_size: int) -> bool:
    xs = [point[0] for point in quad]
    ys = [point[1] for point in quad]
    first_x = max(0, math.ceil((min(xs) - stride / 2) / stride))
    last_x = min(input_size // stride - 1, math.floor((max(xs) - stride / 2) / stride))
    first_y = max(0, math.ceil((min(ys) - stride / 2) / stride))
    last_y = min(input_size // stride - 1, math.floor((max(ys) - stride / 2) / stride))
    if first_x > last_x or first_y > last_y:
        return False
    for y_index in range(first_y, last_y + 1):
        y = (y_index + 0.5) * stride
        for x_index in range(first_x, last_x + 1):
            if _inside_convex((x_index + 0.5) * stride, y, quad):
                return True
    return False


def _distance_band(attributes_json: str | None) -> str:
    if not attributes_json:
        return "unavailable"
    try:
        attributes = json.loads(attributes_json)
    except (TypeError, json.JSONDecodeError):
        return "invalid"
    candidates = [
        value
        for key, value in attributes.items()
        if "distance" in str(key).casefold() or str(key).casefold() == "range"
    ]
    if not candidates:
        return "unavailable"
    value = candidates[0]
    if isinstance(value, (int, float)):
        meters = float(value)
    else:
        match = re.search(r"[-+]?\d+(?:\.\d+)?", str(value))
        if not match:
            return str(value).strip().casefold() or "unavailable"
        meters = float(match.group())
    if meters < 10:
        return "0-10m"
    if meters < 20:
        return "10-20m"
    if meters < 40:
        return "20-40m"
    if meters < 80:
        return "40-80m"
    return "80m+"


def _normalizer(taxonomy_path: Path):
    taxonomy = json.loads(taxonomy_path.read_text(encoding="utf-8"))
    identity = set(taxonomy.get("identity_datasets", []))
    mappings = taxonomy.get("dataset_mappings", {})

    def normalize(value: str) -> str:
        return re.sub(
            r"_+", "_", re.sub(r"[^a-zA-Z0-9]+", "_", value.strip().lower())
        ).strip("_")

    def canonical(dataset: str, category: str) -> str:
        normalized = normalize(category)
        if dataset in identity:
            return normalized
        mapped = mappings.get(dataset, {}).get(normalized, normalized)
        if isinstance(mapped, dict):
            return str(mapped.get("concept", normalized))
        if str(mapped).startswith("__"):
            return normalized
        return str(mapped)

    return canonical


def _rows(
    connection: sqlite3.Connection, limit: int | None
) -> Iterable[tuple[Any, ...]]:
    columns = {
        str(row[1]) for row in connection.execute("PRAGMA table_info(annotations)")
    }
    attributes = "a.attributes_json" if "attributes_json" in columns else "NULL"
    query = f"""
        SELECT i.width, i.height, i.source_dataset, a.category_name,
               a.x1, a.y1, a.x2, a.y2, a.quad_json, {attributes}
        FROM annotations AS a
        JOIN images AS i ON i.image_id = a.image_id
        WHERE a.ignore_region = 0
    """
    parameters: tuple[int, ...] = ()
    if limit is not None:
        query += " LIMIT ?"
        parameters = (limit,)
    cursor = connection.execute(query, parameters)
    while batch := cursor.fetchmany(10_000):
        yield from batch


def analyze_index(
    index_path: Path,
    config: Phase3Config,
    taxonomy_path: Path,
    *,
    limit: int | None = None,
) -> dict[str, Any]:
    canonical = _normalizer(taxonomy_path)
    groups: dict[tuple[str, str, str], Aggregate] = defaultdict(Aggregate)
    strides = config.assignment.strides
    references = [stride * 4.0 for stride in strides]
    with sqlite3.connect(
        f"file:{index_path.resolve()}?mode=ro", uri=True
    ) as connection:
        for row in _rows(connection, limit):
            width, height = int(row[0]), int(row[1])
            dataset, raw_category = str(row[2]), str(row[3])
            category = canonical(dataset, raw_category)
            x1, y1, x2, y2 = map(float, row[4:8])
            quad = [[float(value) for value in point] for point in json.loads(row[8])]
            scale = min(config.data.input_size / width, config.data.input_size / height)
            offset_x = (config.data.input_size - width * scale) / 2.0
            offset_y = (config.data.input_size - height * scale) / 2.0
            model_quad = [
                [point[0] * scale + offset_x, point[1] * scale + offset_y]
                for point in quad
            ]
            source_edges = _edge_lengths(quad)
            model_edges = _edge_lengths(model_quad)
            source_area = _quad_area(quad)
            model_area = _quad_area(model_quad)
            source_short = min(source_edges)
            model_short = min(model_edges)
            model_major = max(model_edges)
            aspect = model_major / max(model_short, 1e-7)
            bbox_width = max(0.0, (x2 - x1) * scale)
            bbox_height = max(0.0, (y2 - y1) * scale)
            hbb_retained = (
                bbox_width * bbox_height >= config.data.tiny_area
                and min(bbox_width, bbox_height) >= config.data.tiny_min_side
            )
            quad_retained = model_short >= config.data.quad_regular_min_side or (
                model_major >= config.data.thin_major_axis_min
                and aspect >= config.data.thin_aspect_ratio_min
                and model_area >= config.data.thin_area
            )
            opportunities = {
                f"P{level + 3}": _grid_opportunity(
                    model_quad, stride, config.data.input_size
                )
                for level, stride in enumerate(strides)
            }
            scenarios = {"p2": _grid_opportunity(model_quad, 4, config.data.input_size)}
            scenarios["p2_min4"] = model_short >= 4.0 or (
                model_major >= config.data.thin_major_axis_min
                and aspect >= config.data.thin_aspect_ratio_min
                and model_area >= config.data.thin_area
            )
            for scenario_size in (512, 640):
                ratio = scenario_size / config.data.input_size
                scenario_quad = [
                    [point[0] * ratio, point[1] * ratio] for point in model_quad
                ]
                scenario_short = model_short * ratio
                scenario_major = model_major * ratio
                scenario_area = model_area * ratio * ratio
                scenario_aspect = scenario_major / max(scenario_short, 1e-7)
                scenarios[f"quad_{scenario_size}"] = (
                    scenario_short >= config.data.quad_regular_min_side
                    or (
                        scenario_major >= config.data.thin_major_axis_min
                        and scenario_aspect >= config.data.thin_aspect_ratio_min
                        and scenario_area >= config.data.thin_area
                    )
                )
                scenarios[f"p3_{scenario_size}"] = _grid_opportunity(
                    scenario_quad, strides[0], scenario_size
                )
            object_scale = math.sqrt(max(model_area, 1e-12))
            level_order = sorted(
                range(len(strides)),
                key=lambda index: abs(math.log(object_scale / references[index])),
            )
            primary_level = f"P{level_order[0] + 3}"
            eligible_levels = "+".join(
                f"P{index + 3}" for index in level_order[: config.quad.eligible_levels]
            )
            values = {
                "source_short_side_px": source_short,
                "model_short_side_px": model_short,
                "source_area_px2": source_area,
                "aspect_ratio": max(aspect, 1.0),
            }
            distance = _distance_band(row[9])
            for key in (
                ("overall", "all", "all"),
                ("dataset", dataset, "all"),
                ("category", "all", category),
                ("dataset_category", dataset, category),
            ):
                groups[key].add(
                    values,
                    hbb=hbb_retained,
                    quad=quad_retained,
                    opportunities=opportunities,
                    scenarios=scenarios,
                    primary_level=primary_level,
                    eligible_levels=eligible_levels,
                    distance_band=distance,
                )
    return {
        "schema_version": "small-object-policy-audit.v1",
        "source_index": str(index_path.resolve()),
        "limit": limit,
        "model_contract": {
            "input_size": config.data.input_size,
            "strides": list(strides),
            "hbb_tiny_area": config.data.tiny_area,
            "hbb_tiny_min_side": config.data.tiny_min_side,
            "quad_regular_min_side": config.data.quad_regular_min_side,
            "quad_thin_major_axis_min": config.data.thin_major_axis_min,
            "quad_thin_aspect_ratio_min": config.data.thin_aspect_ratio_min,
            "quad_thin_area": config.data.thin_area,
            "quad_eligible_levels": config.quad.eligible_levels,
            "counterfactuals": {
                "384_p2_stride": 4,
                "384_p2_regular_min_side": 4,
                "512_area_compute_multiplier": (512 / config.data.input_size) ** 2,
                "640_area_compute_multiplier": (640 / config.data.input_size) ** 2,
            },
        },
        "groups": _serialize_groups(groups),
    }


def _serialize_groups(
    groups: dict[tuple[str, str, str], Aggregate],
) -> list[dict[str, Any]]:
    output = []
    for (scope, dataset, category), aggregate in sorted(groups.items()):
        histograms = {}
        for metric, buckets in aggregate.histograms.items():
            edges = HISTOGRAM_BINS[metric]
            histograms[metric] = [
                {
                    "lower": edges[index],
                    "upper": (
                        "inf" if math.isinf(edges[index + 1]) else edges[index + 1]
                    ),
                    **bucket.__dict__,
                }
                for index, bucket in enumerate(buckets)
            ]
        output.append(
            {
                "scope": scope,
                "dataset": dataset,
                "category": category,
                **aggregate.totals.__dict__,
                "fpn_primary": dict(sorted(aggregate.fpn_primary.items())),
                "fpn_eligible": dict(sorted(aggregate.fpn_eligible.items())),
                "distance_bands": dict(sorted(aggregate.distance_bands.items())),
                "histograms": histograms,
            }
        )
    return output


def write_report(report: dict[str, Any], output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    (output / "small_object_policy.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    fields = [
        "scope",
        "dataset",
        "category",
        "metric",
        "lower",
        "upper",
        "objects",
        "hbb_retained",
        "quad_retained",
        "p3_opportunity",
        "any_grid_opportunity",
        "quad_retained_p3",
        "quad_retained_any_grid",
        "p2_opportunity",
        "quad_retained_p2",
        "p2_min4_retained",
        "p2_min4_retained_grid",
        "quad_retained_512",
        "quad_retained_p3_512",
        "quad_retained_640",
        "quad_retained_p3_640",
    ]
    with (output / "histograms.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for group in report["groups"]:
            for metric, buckets in group["histograms"].items():
                for bucket in buckets:
                    writer.writerow(
                        {
                            **{key: group[key] for key in fields[:3]},
                            "metric": metric,
                            **bucket,
                        }
                    )
    _write_html(report, output / "index.html")


def _percentage(value: int, total: int) -> str:
    return f"{100.0 * value / max(total, 1):.2f}%"


def _write_html(report: dict[str, Any], path: Path) -> None:
    groups = report["groups"]
    overall = next(group for group in groups if group["scope"] == "overall")
    histogram_sections = []
    for metric, buckets in overall["histograms"].items():
        maximum = max((bucket["objects"] for bucket in buckets), default=1)
        bucket_rows = []
        for bucket in buckets:
            label = f"{bucket['lower']}–{bucket['upper']}"
            width = 100.0 * bucket["objects"] / max(maximum, 1)
            bucket_rows.append(
                "<tr><td>{}</td><td>{:,}</td><td><div class='bar' style='width:{:.2f}%'>"
                "</div></td><td>{}</td><td>{}</td></tr>".format(
                    html.escape(label),
                    bucket["objects"],
                    width,
                    _percentage(bucket["quad_retained_p3"], bucket["objects"]),
                    _percentage(bucket["p2_min4_retained_grid"], bucket["objects"]),
                )
            )
        histogram_sections.append(
            "<section><h2>{}</h2><table><thead><tr><th>Bin</th><th>Objects</th>"
            "<th>Distribution</th><th>Current retained+P3</th>"
            "<th>P2 min-4 retained+grid</th></tr></thead><tbody>{}</tbody>"
            "</table></section>".format(html.escape(metric), "".join(bucket_rows))
        )
    rows = []
    for group in groups:
        if group["scope"] not in {"overall", "dataset", "category"}:
            continue
        rows.append(
            "<tr><td>{}</td><td>{}</td><td>{}</td><td>{:,}</td>"
            "<td>{}</td><td>{}</td><td>{}</td><td>{}</td><td>{}</td>"
            "<td>{}</td><td>{}</td><td>{}</td></tr>".format(
                html.escape(group["scope"]),
                html.escape(group["dataset"]),
                html.escape(group["category"]),
                group["objects"],
                _percentage(group["hbb_retained"], group["objects"]),
                _percentage(group["quad_retained"], group["objects"]),
                _percentage(group["p3_opportunity"], group["objects"]),
                _percentage(group["any_grid_opportunity"], group["objects"]),
                _percentage(group["p2_opportunity"], group["objects"]),
                _percentage(group["p2_min4_retained_grid"], group["objects"]),
                _percentage(group["quad_retained_p3_512"], group["objects"]),
                _percentage(group["quad_retained_p3_640"], group["objects"]),
            )
        )
    path.write_text(
        "<!doctype html><meta charset='utf-8'><title>Small-object policy audit</title>"
        "<style>body{font:14px sans-serif;margin:2rem;background:#111;color:#eee}"
        "table{border-collapse:collapse;width:100%}td,th{border:1px solid #555;"
        "padding:.35rem;text-align:right}td:nth-child(-n+3),th:nth-child(-n+3)"
        "{text-align:left}tr:nth-child(even){background:#222}.bar{height:1rem;"
        "background:#3b82f6;min-width:1px}section{margin:2rem 0}</style>"
        "<h1>Small-object policy audit</h1><p>Retention is measured after aspect-"
        "preserving letterbox. Grid opportunity means at least one cell center lies "
        "inside the transformed quad.</p>"
        + "".join(histogram_sections)
        + "<h2>Group summary</h2><table><thead><tr><th>Scope</th><th>Dataset"
        "</th><th>Category</th><th>Objects</th><th>HBB retained</th><th>Quad retained"
        "</th><th>P3 opportunity</th><th>Any-grid opportunity</th><th>P2 opportunity"
        "</th><th>P2 min-4 retained+grid</th><th>512 retained+P3</th>"
        "<th>640 retained+P3</th></tr></thead><tbody>"
        + "".join(rows)
        + "</tbody></table>",
        encoding="utf-8",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/phase3.yaml"))
    parser.add_argument("--index", type=Path)
    parser.add_argument(
        "--taxonomy", type=Path, default=Path("automotive_taxonomy_mapping.json")
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--limit", type=int)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = load_phase3_config(args.config)
    index = args.index or config.data.index_dir / "quad_train.sqlite"
    report = analyze_index(index, config, args.taxonomy, limit=args.limit)
    write_report(report, args.output)
    overall = next(group for group in report["groups"] if group["scope"] == "overall")
    print(
        json.dumps(
            {key: value for key, value in overall.items() if key != "histograms"},
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
