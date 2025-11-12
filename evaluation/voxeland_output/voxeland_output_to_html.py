#!/usr/bin/env python3
"""
Interactive 3D Voxeland Semantic Map Visualizer (multi-scene, multi-detector)

Folder layout (this script's directory):
  ./voxeland_output_to_html.py
  ./scannet_scene0000_01/
      voxeland_semantic_map_detectron_scannet_scene0000_01.ply
      voxeland_semantic_map_talos_scannet_scene0000_01.ply
      voxeland_semantic_map_yoloe_scannet_scene0000_01.ply
  ./scenenn_011/
      voxeland_semantic_map_detectron_scenenn_011.ply
      ...

Usage examples:
  # Process ALL scenes and ALL detectors
  python3 voxeland_output_to_html.py

  # Process one scene (all detectors)
  python3 voxeland_output_to_html.py -s scannet_scene0000_01

  # Process one scene + one detector
  python3 voxeland_output_to_html.py -s scannet_scene0000_01 -d detectron

  # Error (detector alone is not allowed)
  python3 voxeland_output_to_html.py -d detectron

  # Hide "unknown" (filename will end with _nu.html)
  python3 voxeland_output_to_html.py -s scannet_scene0000_01 -d detectron -u
"""

import argparse
import sys
import re
from pathlib import Path
from collections import defaultdict

import numpy as np
import plotly.graph_objects as go


DETECTORS = ("detectron", "talos", "yoloe")
# Updated pattern to match both scannet_scene####_## and scenenn_###
SCENE_DIR_PATTERN = re.compile(r"^(scannet_scene\d{4}_\d{2}|scenenn_\d{3})$")


def read_ply_file(ply_path: Path):
    """Reads a Voxeland ASCII PLY file with semantic information."""
    print(f"[Read] PLY: {ply_path}")
    vertices = []
    categories = []
    header_lines = []
    reading_header = True

    with open(ply_path, "r") as f:
        for line in f:
            line = line.strip()

            if reading_header:
                header_lines.append(line)
                if line == "end_header":
                    reading_header = False
                continue

            parts = line.split()
            if len(parts) >= 8:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                r, g, b = int(parts[3]), int(parts[4]), int(parts[5])
                instance_id = int(parts[6])
                # Category can have multiple words (e.g., "washing machine")
                # Join all remaining parts from index 7 onwards
                category = ' '.join(parts[7:])
                vertices.append([x, y, z, r, g, b, instance_id])
                categories.append(category)

    vertices_array = np.array(vertices, dtype=float) if vertices else np.zeros((0, 7), dtype=float)
    print(f"  ✓ {len(vertices_array)} vertices")

    header_info = {}
    for line in header_lines:
        if line.startswith("comment "):
            header_info.setdefault("comments", []).append(line.replace("comment ", ""))

    return {"vertices": vertices_array, "categories": categories, "header_info": header_info}


def organize_by_instances(vertices: np.ndarray, categories: list[str]):
    """
    Groups vertices by instance_id and determines the dominant category per instance.
    """
    inst = defaultdict(lambda: {
        "coords": [],
        "colors": [],
        "category_counts": defaultdict(int),
        "count": 0
    })

    for i, v in enumerate(vertices):
        x, y, z, r, g, b, instance_id = v
        cat = categories[i] if i < len(categories) else "unknown"
        inst[instance_id]["coords"].append([x, y, z])
        inst[instance_id]["colors"].append([r, g, b])
        inst[instance_id]["category_counts"][cat] += 1
        inst[instance_id]["count"] += 1

    result = {}
    for instance_id, data in inst.items():
        coords = np.asarray(data["coords"], dtype=float)
        colors = np.asarray(data["colors"], dtype=float)
        dominant_category = max(data["category_counts"].items(), key=lambda x: x[1])[0]
        result[int(instance_id)] = {
            "coords": coords,
            "colors": colors,
            "category": dominant_category,
            "count": data["count"],
            "category_counts": dict(data["category_counts"])
        }
    return result


def create_interactive_visualization(ply_data, output_html: Path, show_unknown: bool = True):
    """
    Builds and saves an interactive Plotly 3D visualization.
    - Unknown instances (category == 'unknown') are shown in gray (no subsampling).
    - Known instances are colored by their average RGB.
    """
    vertices = ply_data["vertices"]
    categories = ply_data["categories"]

    print("[Info] Organizing by instances...")
    instances = organize_by_instances(vertices, categories)

    fig = go.Figure()

    unknown_instances = {k: v for k, v in instances.items() if v["category"] == "unknown"}
    known_instances = {k: v for k, v in instances.items() if v["category"] != "unknown"}

    print(f"  Unknown instances: {len(unknown_instances)}")
    print(f"  Known instances:   {len(known_instances)}")

    # Unknown instances (gray)
    if show_unknown and unknown_instances:
        for inst_id, data in unknown_instances.items():
            coords = data["coords"]
            if coords.size == 0:
                continue
            fig.add_trace(go.Scatter3d(
                x=coords[:, 0], y=coords[:, 1], z=coords[:, 2],
                mode="markers",
                name=f"Unknown (ID:{inst_id})",
                marker=dict(size=2, color="rgb(120,120,120)", opacity=0.35),
                text=[f"Unknown<br>Instance: {inst_id}"] * len(coords),
                hovertemplate="<b>%{text}</b><br>X:%{x:.2f} Y:%{y:.2f} Z:%{z:.2f}<extra></extra>",
                showlegend=False
            ))

    # Known instances (average color)
    for inst_id, data in sorted(known_instances.items(), key=lambda x: x[1]["count"], reverse=True):
        coords = data["coords"]
        colors = data["colors"]
        if coords.size == 0:
            continue
        avg = np.mean(colors, axis=0).astype(int) if colors.size else np.array([200, 200, 200])
        color_str = f"rgb({avg[0]}, {avg[1]}, {avg[2]})"
        hover_text = [f"<b>{data['category']}</b><br>Instance ID: {inst_id}<br>Vertices: {data['count']}"] * len(coords)
        fig.add_trace(go.Scatter3d(
            x=coords[:, 0], y=coords[:, 1], z=coords[:, 2],
            mode="markers",
            name=f"{data['category']} (ID:{inst_id})",
            marker=dict(size=3, color=color_str, opacity=0.8, line=dict(width=0)),
            text=hover_text,
            hovertemplate="%{text}<br>X:%{x:.2f} Y:%{y:.2f} Z:%{z:.2f}<extra></extra>",
        ))

    total_vertices = len(vertices)
    num_instances = len(instances)

    fig.update_layout(
        title=dict(
            text=f'Voxeland Semantic Map<br><sub>{total_vertices:,} vertices | {num_instances} instances</sub>',
            x=0.5, xanchor="center", font=dict(color="white")
        ),
        scene=dict(
            xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)",
            aspectmode="data",
            camera=dict(eye=dict(x=1.5, y=1.5, z=1.5)),
            xaxis=dict(backgroundcolor="rgb(20,20,20)", gridcolor="rgb(60,60,60)",
                       showbackground=True, zerolinecolor="rgb(80,80,80)",
                       title=dict(font=dict(color="white")), tickfont=dict(color="white")),
            yaxis=dict(backgroundcolor="rgb(20,20,20)", gridcolor="rgb(60,60,60)",
                       showbackground=True, zerolinecolor="rgb(80,80,80)",
                       title=dict(font=dict(color="white")), tickfont=dict(color="white")),
            zaxis=dict(backgroundcolor="rgb(20,20,20)", gridcolor="rgb(60,60,60)",
                       showbackground=True, zerolinecolor="rgb(80,80,80)",
                       title=dict(font=dict(color="white")), tickfont=dict(color="white")),
        ),
        width=1400, height=900,
        showlegend=True,
        legend=dict(
            yanchor="top", y=0.99, xanchor="left", x=0.01,
            bgcolor="rgba(30,30,30,0.9)", bordercolor="rgb(60,60,60)",
            borderwidth=1, font=dict(color="white")
        ),
        hovermode="closest",
        paper_bgcolor="rgb(15,15,15)",
        plot_bgcolor="rgb(15,15,15)",
    )

    output_html.parent.mkdir(parents=True, exist_ok=True)
    print(f"[Save] {output_html}")
    fig.write_html(str(output_html))


def expected_ply_path(script_dir: Path, scene_name: str, detector: str) -> Path:
    """Returns the expected PLY path for a (scene_name, detector) pair."""
    scene_dir = script_dir / scene_name
    fname = f"voxeland_semantic_map_{detector}_{scene_name}.ply"
    return scene_dir / fname


def output_html_path(script_dir: Path, scene_name: str, detector: str, no_unknown: bool = False) -> Path:
    """Returns the output HTML path; saved inside the scene directory.
       If no_unknown=True, filename ends with _nu.html."""
    scene_dir = script_dir / scene_name
    suffix = "_nu" if no_unknown else ""
    return scene_dir / f"voxeland_map_3d_{detector}_{scene_name}{suffix}.html"


def list_all_scene_names(script_dir: Path) -> list[str]:
    """Scans this folder for scene directories and returns their names."""
    scene_names = []
    for p in sorted(script_dir.iterdir()):
        if p.is_dir() and SCENE_DIR_PATTERN.match(p.name):
            scene_names.append(p.name)
    return scene_names


def main():
    parser = argparse.ArgumentParser(
        description="Interactive 3D visualizer for Voxeland semantic maps (multi-scene/multi-detector)"
    )
    parser.add_argument("-s", "--scene", type=str, default=None,
                        help='Scene name, e.g., "scannet_scene0000_01" or "scenenn_011"')
    parser.add_argument("-d", "--detector", type=str, choices=DETECTORS, default=None,
                        help='Detector name: one of {"detectron","talos","yoloe"}')
    parser.add_argument("-u", "--no-unknown", action="store_true",
                        help='Hide vertices with category "unknown" (output filename ends with _nu.html)')

    args = parser.parse_args()
    script_dir = Path(__file__).parent.resolve()

    # Validate combinations:
    if args.detector and not args.scene:
        print("ERROR: --detector requires --scene. Provide a scene name (e.g., -s scannet_scene0000_01).", file=sys.stderr)
        sys.exit(2)

    # Build worklist of (scene_name, detector)
    worklist: list[tuple[str, str]] = []

    if args.scene and args.detector:
        worklist = [(args.scene, args.detector)]
    elif args.scene and not args.detector:
        # Process all detectors for that scene
        worklist = [(args.scene, d) for d in DETECTORS]
    else:
        # Neither scene nor detector -> process all scenes and all detectors
        all_scenes = list_all_scene_names(script_dir)
        if not all_scenes:
            print("No scene directories found (expected names like 'scannet_scene0000_01' or 'scenenn_011').", file=sys.stderr)
            sys.exit(1)
        for scene_name in all_scenes:
            worklist.extend((scene_name, d) for d in DETECTORS)

    any_ok = False
    for scene_name, detector in worklist:
        # Determine output HTML path first; if it already exists, skip processing early.
        out_html = output_html_path(script_dir, scene_name, detector, args.no_unknown)
        if out_html.exists():
            print(f"[Skip] Output already exists, skipping: {out_html}")
            any_ok = True  # consider as handled
            continue

        # Check input PLY
        ply_path = expected_ply_path(script_dir, scene_name, detector)
        if not ply_path.exists():
            print(f"[Skip] Missing PLY for scene {scene_name} / detector {detector}: {ply_path}")
            continue

        try:
            ply_data = read_ply_file(ply_path)
            create_interactive_visualization(
                ply_data=ply_data,
                output_html=out_html,
                show_unknown=not args.no_unknown
            )
            print(f"[OK] Scene {scene_name} / {detector} -> {out_html}\n")
            any_ok = True
        except Exception as e:
            print(f"[Error] Scene {scene_name} / {detector}: {e}", file=sys.stderr)

    if not any_ok:
        print("Nothing was processed. Check paths and file names.", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
