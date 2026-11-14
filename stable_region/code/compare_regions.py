#!/usr/bin/env python3
"""Plot exact accepted-cell unions; never fit, interpolate, or align regions.

Usage:
  python compare_regions.py
  python compare_regions.py --output-dir OUT --carsim-fullcar-dir DIR

Default outputs: stable_region/comparison/images and comparison/metrics.
An explicit --output-dir stores both figures and metrics there unless --metrics-dir is given.

The prediction directory should contain carsim_fullcar_region_{0,1p5,2p5}_core.mat
with RESEARCH2D.CASE.{grid,masks,region}, or NPZ files with vc, r, cell_mask,
point_mask, reference_mask. Missing predictions produce an explicitly labelled
two-model interim comparison, never a fabricated third region.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import hashlib
import itertools
import json
import math
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(os.environ.get("TMPDIR", "/tmp")) / "stable-region-matplotlib"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.lines import Line2D
import numpy as np
from scipy.io import loadmat

BASE = Path(__file__).resolve().parents[1]
KEYS = ("0", "1p5", "2p5")
ANGLES = (0.0, 1.5, 2.5)
MODELS = ("carsim", "carsim_fullcar", "simul_fullcar")
STYLE = {
    "carsim": dict(color="#171717", linestyle="solid", linewidth=3.2),
    "carsim_fullcar": dict(color="#0875cf", linestyle="solid", linewidth=1.75),
    "simul_fullcar": dict(color="#e47f12", linestyle=(0, (5, 2.8)), linewidth=2.0),
}


@dataclass
class Region:
    model: str
    source: Path
    vc: np.ndarray
    r: np.ndarray
    cells: np.ndarray
    points: np.ndarray
    reference: np.ndarray

    @property
    def weights(self):
        return np.diff(self.vc)[:, None] * np.diff(self.r)[None, :]

    @property
    def area(self):
        return float(np.sum(self.weights[self.cells]))


def load_region(path: Path, model: str) -> Region:
    if path.suffix == ".npz":
        with np.load(path, allow_pickle=False) as d:
            region = Region(model, path, d["vc"].ravel(), d["r"].ravel(),
                            d["cell_mask"].astype(bool), d["point_mask"].astype(bool),
                            d["reference_mask"].astype(bool))
    else:
        data = loadmat(path, simplify_cells=True)["RESEARCH2D"]
        c = data["CASE"]
        if not isinstance(c, dict):
            assert np.size(c) == 1
            c = np.asarray(c, dtype=object).ravel()[0]
        points = np.asarray(c["masks"]["final"], dtype=bool)
        if points.ndim == 3:
            points = points[:, :, -1]
        reference = c["masks"].get("reference", c.get("handling", {}).get("mask"))
        region = Region(model, path, np.asarray(c["grid"]["vc"]).ravel(),
                        np.asarray(c["grid"]["r"]).ravel(),
                        np.asarray(c["region"]["cell_mask"], dtype=bool), points,
                        np.asarray(reference, dtype=bool))
    assert np.all(np.diff(region.vc) > 0) and np.all(np.diff(region.r) > 0)
    assert region.cells.shape == (len(region.vc) - 1, len(region.r) - 1)
    assert region.points.shape == region.reference.shape == (len(region.vc), len(region.r))
    assert not np.any(region.points & ~region.reference)
    return region


def exposed_edges(mask: np.ndarray):
    """Directed grid-index edges, accepted cell on the left side."""
    edges = set()
    nv, nr = mask.shape
    for i, j in np.argwhere(mask):
        i, j = int(i), int(j)
        if j == 0 or not mask[i, j - 1]: edges.add(((i, j), (i + 1, j)))
        if i == nv - 1 or not mask[i + 1, j]: edges.add(((i + 1, j), (i + 1, j + 1)))
        if j == nr - 1 or not mask[i, j + 1]: edges.add(((i + 1, j + 1), (i, j + 1)))
        if i == 0 or not mask[i - 1, j]: edges.add(((i, j + 1), (i, j)))
    return edges


def boundary_paths(region: Region):
    """Join only existing exposed edges; preserve holes and diagonal contacts."""
    edges = exposed_edges(region.cells)
    remaining = set(edges)
    outgoing = {}
    for a, b in edges:
        outgoing.setdefault(a, set()).add(b)
    paths = []
    while remaining:
        first = min(remaining)
        remaining.remove(first)
        indices = [first[0], first[1]]
        while indices[-1] != indices[0]:
            prev, at = indices[-2:]
            choices = [b for b in outgoing.get(at, ()) if (at, b) in remaining]
            if not choices: raise AssertionError("An exposed-cell boundary is open")
            direction = (at[0] - prev[0], at[1] - prev[1])
            def turn(b):
                nxt = (b[0] - at[0], b[1] - at[1])
                return math.atan2(direction[0] * nxt[1] - direction[1] * nxt[0],
                                  direction[0] * nxt[0] + direction[1] * nxt[1])
            b = max(choices, key=turn)
            remaining.remove((at, b)); indices.append(b)
        paths.append(np.array([(region.vc[i], region.r[j]) for i, j in indices]))
    actual_count = sum(len(p) - 1 for p in paths)
    assert actual_count == len(edges)
    signed_area = sum(.5 * np.sum(p[:-1, 0] * p[1:, 1] - p[1:, 0] * p[:-1, 1]) for p in paths)
    assert np.isclose(signed_area, region.area, rtol=2e-12, atol=1e-13)
    return paths


def geometry_self_test():
    cases = [(np.ones((1, 1), bool), 4, 1),
             (np.ones((2, 2), bool), 8, 4),
             (np.eye(2, dtype=bool), 8, 2)]
    hole = np.ones((3, 3), bool); hole[1, 1] = False
    cases.append((hole, 16, 8))
    for cells, edge_count, area in cases:
        n, m = cells.shape
        fake = Region("test", Path("synthetic"), np.arange(n + 1), np.arange(m + 1),
                      cells, np.zeros((n + 1, m + 1), bool), np.ones((n + 1, m + 1), bool))
        assert len(exposed_edges(cells)) == edge_count and fake.area == area
        boundary_paths(fake)


def ratio(a, b, *, empty=1.0): return float(a / b) if b else empty


def pair_metrics(a: Region, b: Region):
    assert a.cells.shape == b.cells.shape
    assert np.allclose(a.vc, b.vc, atol=2e-12, rtol=0)
    assert np.allclose(a.r, b.r, atol=2e-12, rtol=0)
    intersection = int(np.count_nonzero(a.cells & b.cells))
    union = int(np.count_nonzero(a.cells | b.cells))
    common_h = a.reference & b.reference
    ap, bp = a.points[common_h], b.points[common_h]
    return {
        "models": [a.model, b.model], "accepted_cells": [int(a.cells.sum()), int(b.cells.sum())],
        "cell_intersection": intersection, "cell_union": union,
        "cell_iou": ratio(intersection, union),
        "cell_dice": ratio(2 * intersection, int(a.cells.sum() + b.cells.sum())),
        "cell_disagreement": int(np.count_nonzero(a.cells ^ b.cells)),
        "areas": [a.area, b.area], "signed_area_difference_b_minus_a": b.area - a.area,
        "area_difference_pct_of_a": ratio(100 * (b.area - a.area), a.area, empty=None),
        "reference_mask_disagreement": int(np.count_nonzero(a.reference ^ b.reference)),
        "common_reference_points": int(common_h.sum()),
        "point_agreement_on_common_reference": ratio(int(np.count_nonzero(ap == bp)), len(ap), empty=None),
        "point_iou_on_common_reference": ratio(int(np.count_nonzero(ap & bp)), int(np.count_nonzero(ap | bp))),
        "grid_axis_max_difference": {"vc": float(np.max(np.abs(a.vc - b.vc))),
                                     "r": float(np.max(np.abs(a.r - b.r)))},
        "comparison_method": "index correspondence on equal nominal grids; no interpolation or alignment",
        "same_reference_mask": bool(np.array_equal(a.reference,b.reference)),
        "recovery_test_scope": ("same observed-seven-signal comparison criterion" if "simul_fullcar" not in (a.model,b.model)
                                else "exact recovery-criterion equivalence of the supplied simul_fullcar LUT is not established"),
    }


def add_region(ax, region: Region):
    color = STYLE[region.model]["color"]
    rectangles = []
    for i, j in np.argwhere(region.cells):
        rectangles.append([(region.vc[i], region.r[j]), (region.vc[i + 1], region.r[j]),
                           (region.vc[i + 1], region.r[j + 1]), (region.vc[i], region.r[j + 1])])
    ax.add_collection(PolyCollection(rectangles, facecolors=color, edgecolors="none", alpha=.055))
    ax.add_collection(LineCollection(boundary_paths(region), **STYLE[region.model], zorder=4))


def shared_limits(cases, full_grid):
    all_regions = [v for case in cases for v in case.values()]
    first = all_regions[0]
    if full_grid:
        return (first.vc[0], first.vc[-1]), (first.r[0], first.r[-1])
    xs, ys = [], []
    for reg in all_regions:
        ids = np.argwhere(reg.cells)
        if len(ids):
            xs.extend([reg.vc[ids[:, 0].min()], reg.vc[ids[:, 0].max() + 1]])
            ys.extend([reg.r[ids[:, 1].min()], reg.r[ids[:, 1].max() + 1]])
    if not xs: return (first.vc[0], first.vc[-1]), (first.r[0], first.r[-1])
    return ((math.floor((min(xs) - .15) / .5) * .5, math.ceil((max(xs) + .15) / .5) * .5),
            (math.floor((min(ys) - .08) / .25) * .25, math.ceil((max(ys) + .08) / .25) * .25))


def panel(ax, angle, regions, limits):
    for name in MODELS:
        if name in regions: add_region(ax, regions[name])
    ax.set(xlim=limits[0], ylim=limits[1], xlabel=r"$v_{y,0}$ command (m/s)",
           ylabel=r"$r_0$ command (rad/s)", title=f"Steering command {angle:g}°")
    ax.axhline(0, color="#b6bdc5", linewidth=.6, zorder=0)
    ax.axvline(0, color="#b6bdc5", linewidth=.6, zorder=0)
    ax.grid(color="#d9dee5", linewidth=.55, alpha=.65)
    ax.set_axisbelow(True)
    area_text = "Cell area [(m/s)·(rad/s)]\n" + "\n".join(f"{m}: {regions[m].area:.4f}" for m in MODELS if m in regions)
    ax.text(.025, .025, area_text, transform=ax.transAxes, va="bottom", ha="left", fontsize=9.5,
            bbox=dict(boxstyle="round,pad=.35", facecolor="white", edgecolor="#d1d8e0", alpha=.92))


def figure(cases, angles, limits, output, complete):
    count = len(cases)
    fig, axes = plt.subplots(1, count, figsize=(6.3 * count, 7.0), squeeze=False)
    for ax, case, angle in zip(axes[0], cases, angles): panel(ax, angle, case, limits)
    names = [name for name in MODELS if name in cases[0]]
    title = "Stability-region comparison" if complete else "Two supplied regions — interim comparison"
    fig.suptitle(title + "\n70 km/h  |  μ = 0.6  |  wear [25,25,0,0]  |  10 s", fontsize=15, y=.985)
    handles = [Line2D([0], [0], label=name, **STYLE[name]) for name in names]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(.5, .865), ncol=len(names), frameon=False, fontsize=11)
    if count == 1:
        foot = ("Exact accepted-cell union boundaries; nominal initial-condition grid.\n"
                "H: carsim/carsim_fullcar use measured wheel steer; simul_fullcar uses nominal steer.\n"
                "Return: carsim/carsim_fullcar use 7 signals; supplied LUT equivalence is not established.")
        left, right = .12, .97
    else:
        foot = ("Exact accepted-cell union boundaries on nominal IC coordinates; no convex hull, geometric shifting, or interpolation.\n"
                "H reference: carsim/carsim_fullcar use measured wheel steer −0.007° / 1.308° / 2.206°; simul_fullcar uses nominal 0° / 1.5° / 2.5°.\n"
                "Return: carsim/carsim_fullcar use 7 observed signals; exact recovery-criterion equivalence of the supplied simul_fullcar LUT is not established.")
        left, right = .055, .99
    fig.text(.5, .035, foot, ha="center", va="bottom", fontsize=9.1, color="#45515f", linespacing=1.55)
    fig.subplots_adjust(left=left, right=right, bottom=.22, top=.775, wspace=.23)
    fig.savefig(output.with_suffix(".png"), dpi=160, facecolor="white")
    fig.savefig(output.with_suffix(".svg"), facecolor="white")
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output-dir", type=Path, default=None)
    p.add_argument("--metrics-dir", type=Path, default=None)
    p.add_argument("--carsim-dir", type=Path, default=BASE / "comparison/reference_core")
    p.add_argument("--simul-fullcar-dir", type=Path, default=BASE / "comparison/reference_core")
    p.add_argument("--carsim-fullcar-dir", type=Path, default=BASE / "comparison/carsim_fullcar")
    p.add_argument("--keys", nargs="+", choices=KEYS, default=list(KEYS),
                   help="Render only these actual available steering cases")
    p.add_argument("--full-grid", action="store_true")
    args = p.parse_args()
    explicit_output = args.output_dir is not None
    args.output_dir = args.output_dir or BASE / "comparison/images"
    args.metrics_dir = args.metrics_dir or (args.output_dir if explicit_output else BASE / "comparison/metrics")
    geometry_self_test()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.metrics_dir.mkdir(parents=True, exist_ok=True)
    cases, report = [], {"schema": "STABLE_REGION_THREE_MODEL_COMPARISON_V1", "conditions": [],
                        "area_units": "(m/s)*(rad/s)", "cell_boundary_method": "exact exposed cell edges",
                        "point_accuracy_domain": "intersection of evaluated reference masks; outside-H points excluded"}
    selected = [(key, angle) for key, angle in zip(KEYS, ANGLES) if key in set(args.keys)]
    for key, angle in selected:
        regions = {
            "carsim": load_region(args.carsim_dir / f"CarSim_region_{key}_core.mat", "carsim"),
            "simul_fullcar": load_region(args.simul_fullcar_dir / f"simul_fullcar_region_{key}_core.mat", "simul_fullcar"),
        }
        if args.carsim_fullcar_dir:
            candidates = [args.carsim_fullcar_dir / f"carsim_fullcar_region_{key}_core{suffix}" for suffix in (".mat", ".npz")]
            found = next((x for x in candidates if x.is_file()), None)
            if found is None: raise FileNotFoundError(f"No carsim_fullcar region for {angle:g}° in {args.carsim_fullcar_dir}")
            regions["carsim_fullcar"] = load_region(found, "carsim_fullcar")
        for r in regions.values(): boundary_paths(r)
        cases.append(regions)
        report["conditions"].append({"speed_kmh": 70, "mu": .6, "wear": [25,25,0,0],
            "steer_command_deg": angle,
            "models": {name: {"source_file": str(r.source), "source_sha256": hashlib.sha256(r.source.read_bytes()).hexdigest(),
                "area": r.area, "accepted_cells": int(r.cells.sum()), "final_points": int(r.points.sum()),
                "reference_points": int(r.reference.sum())} for name,r in regions.items()},
            "pairwise": [pair_metrics(regions[a],regions[b]) for a,b in itertools.combinations([n for n in MODELS if n in regions],2)]})
    complete = bool(args.carsim_fullcar_dir)
    report["status"] = "complete_three_model_comparison" if complete else "interim_two_supplied_models_only"
    report["reference_difference"] = "carsim H uses measured wheel steer; simul_fullcar H uses nominal command steer"
    report["recovery_definition_difference"] = "carsim/carsim_fullcar use the observed-seven-signal comparison; exact recovery-criterion equivalence of the supplied simul_fullcar LUT is not established"
    limits = shared_limits(cases,args.full_grid)
    report["shared_plot_limits"] = {"vc": limits[0], "r": limits[1]}
    (args.metrics_dir / "comparison_metrics.json").write_text(json.dumps(report,indent=2)+"\n")
    csv_rows = []
    for condition in report["conditions"]:
        for pair in condition["pairwise"]:
            csv_rows.append({"steer_command_deg":condition["steer_command_deg"],
                "model_a":pair["models"][0],"model_b":pair["models"][1],
                "area_a":pair["areas"][0],"area_b":pair["areas"][1],
                "cell_iou":pair["cell_iou"],"cell_dice":pair["cell_dice"],
                "cell_disagreement":pair["cell_disagreement"],
                "reference_mask_disagreement":pair["reference_mask_disagreement"],
                "common_reference_points":pair["common_reference_points"],
                "point_agreement_on_common_reference":pair["point_agreement_on_common_reference"]})
    if csv_rows:
        with (args.metrics_dir/"pairwise_metrics.csv").open("w",newline="") as stream:
            writer=csv.DictWriter(stream,fieldnames=list(csv_rows[0]));writer.writeheader();writer.writerows(csv_rows)
    stem = "three_model_regions" if complete else "two_supplied_models_interim"
    figure(cases,[angle for _,angle in selected],limits,args.output_dir/stem,complete)
    for (key,angle),case in zip(selected,cases): figure([case],[angle],limits,args.output_dir/f"{stem}_{key}deg",complete)
    print(json.dumps({"status":report["status"],"output_dir":str(args.output_dir),"png_count":1+len(selected),
                      "steering_commands":[angle for _,angle in selected]}))


if __name__ == "__main__": main()
