#!/usr/bin/env python3
"""Render PDE2D_MESH v1/v2/v3/v4 topology to PNG, without a browser/server.

Use the project-local visualization venv; see docs/showcase/README.md.
The v3/v4 metadata trailer is intentionally ignored: this is a topology plotter,
not an importer or validator for resuming refinement.
"""
import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D


def read_mesh(path):
    with path.open(encoding="utf-8") as stream:
        def record():
            line = stream.readline()
            if not line:
                raise ValueError(f"{path}: truncated mesh")
            return line.split()

        header = record()
        if len(header) != 2 or header[0] != "PDE2D_MESH" or header[1] not in {"1", "2", "3", "4"}:
            raise ValueError(f"{path}: unsupported mesh header")
        version = int(header[1])
        if record() != ["ORDER", "1"]:
            raise ValueError(f"{path}: expected P1 mesh")

        def count(label):
            row = record()
            if len(row) != 2 or row[0] != label or int(row[1]) < 0:
                raise ValueError(f"{path}: expected {label} count")
            return int(row[1])

        nodes = []
        for _ in range(count("NODES")):
            row = record()
            if len(row) != 2:
                raise ValueError(f"{path}: invalid node")
            point = tuple(map(float, row))
            if not all(math.isfinite(x) for x in point):
                raise ValueError(f"{path}: nonfinite coordinate")
            nodes.append(point)
        triangles = []
        for _ in range(count("ELEMENTS")):
            row = list(map(int, record()))
            if len(row) != 4 or row[0] != 3:
                raise ValueError(f"{path}: expected three-node triangle")
            triangles.append(row[1:])
        boundary, tags = [], []
        for _ in range(count("BOUNDARY_EDGES")):
            row = record()
            if len(row) != (2 if version == 1 else 3):
                raise ValueError(f"{path}: invalid boundary edge")
            boundary.append(tuple(map(int, row[:2])))
            tags.append(row[2] if version >= 2 and row[2] != "-" else "untagged")
        if not nodes or not triangles:
            raise ValueError(f"{path}: empty mesh")
        if any(i < 0 or i >= len(nodes) for row in triangles + boundary for i in row):
            raise ValueError(f"{path}: vertex index out of bounds")
        return nodes, triangles, boundary, tags


def draw(ax, mesh, title, colors):
    nodes, triangles, boundary, tags = mesh
    edges = {tuple(sorted((t[i], t[(i + 1) % 3]))) for t in triangles for i in range(3)}
    ax.add_collection(LineCollection([[nodes[a], nodes[b]] for a, b in sorted(edges)],
                                    colors="#7892aa", linewidths=0.55))
    ax.add_collection(LineCollection([[nodes[a], nodes[b]] for a, b in boundary],
                                    colors=[colors[tag] for tag in tags], linewidths=1.9))
    ax.scatter(*zip(*nodes), s=4, color="#15324b", zorder=3)
    ax.autoscale()
    ax.margins(0.1)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title.replace("_", " ") + f"\n{len(nodes)} vertices · {len(triangles)} triangles",
                 fontsize=9, color="#15324b")
    ax.tick_params(labelsize=7, colors="#667788")
    for spine in ax.spines.values():
        spine.set_color("#dce4ec")
    return set(tags)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("meshes", nargs="*", type=Path)
    parser.add_argument("--manifest", type=Path, help="TSV emitted by showcase.cpp")
    parser.add_argument("--output-dir", type=Path, default=Path("docs/showcase"))
    parser.add_argument("--columns", type=int, default=4)
    args = parser.parse_args()
    if args.columns < 1:
        parser.error("--columns must be positive")
    entries = [(path, path.stem) for path in args.meshes]
    if args.manifest:
        with args.manifest.open(encoding="utf-8") as stream:
            entries.extend((args.manifest.parent / row["file"], row["title"])
                           for row in csv.DictReader(stream, delimiter="\t"))
    if not entries:
        parser.error("supply mesh paths or --manifest")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    loaded = {path: read_mesh(path) for path in dict(entries)}
    all_tags = sorted({tag for mesh in loaded.values() for tag in mesh[3]})
    palette = plt.get_cmap("tab10")
    colors = {tag: palette(i % 10) for i, tag in enumerate(all_tags)}
    colors["untagged"] = "#27384a"
    for path, title in entries:
        fig, ax = plt.subplots(figsize=(5, 4), layout="constrained")
        used = draw(ax, loaded[path], title, colors)
        ax.legend(handles=[Line2D([0], [0], color=colors[tag], label=tag, linewidth=2)
                           for tag in sorted(used)], fontsize=7, loc="best")
        fig.savefig(args.output_dir / (path.stem + ".png"), dpi=160)
        plt.close(fig)
    rows = math.ceil(len(entries) / args.columns)
    fig, axes = plt.subplots(rows, args.columns, figsize=(4 * args.columns, 3.4 * rows),
                             squeeze=False)
    for ax, (path, title) in zip(axes.flat, entries):
        draw(ax, loaded[path], title, colors)
    for ax in list(axes.flat)[len(entries):]:
        ax.set_visible(False)
    fig.suptitle("P1 mesh showcase · boundary colors identify regions", fontsize=16,
                 color="#15324b")
    fig.legend(handles=[Line2D([0], [0], color=colors[tag], label=tag, linewidth=2)
                        for tag in all_tags], loc="lower center", ncol=min(8, len(all_tags)),
               fontsize=9)
    fig.tight_layout(rect=(0, 0.025, 1, 0.975), h_pad=2.3)
    gallery = args.output_dir / "gallery.png"
    fig.savefig(gallery, dpi=150, facecolor="white")
    plt.close(fig)
    print(f"Wrote {len(entries)} individual PNGs and {gallery}")


if __name__ == "__main__":
    main()
