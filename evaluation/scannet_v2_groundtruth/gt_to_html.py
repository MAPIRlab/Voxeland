#!/usr/bin/env python3
"""
Interactive 3D ScanNet Ground Truth Visualizer

This script visualizes ScanNet ground truth data in the same HTML format as the 
Voxeland predictions. It reads the original ScanNet format:
  - scene_*.ply (binary PLY with x,y,z,r,g,b,alpha and faces)
  - *.aggregation.json (instance labels and segment groups)
  - *.segs.json (vertex to segment mapping)

Usage examples:
  # Process ALL scenes in the directory
  python3 gt_to_html.py

  # Process one specific scene
  python3 gt_to_html.py -s scene0000_01

  # Hide "unlabeled" vertices (filename will end with _nu.html)
  python3 gt_to_html.py -s scene0000_01 -u
"""

import argparse
import sys
import json
from pathlib import Path
from collections import defaultdict
import struct

import numpy as np
import plotly.graph_objects as go


def read_aggregation_json(agg_path: Path) -> dict:
    """
    Reads the *.aggregation.json file.
    Returns a dict: segment_id -> (object_id, label)
    """
    print(f"[Read] Aggregation: {agg_path}")
    with open(agg_path, "r") as f:
        data = json.load(f)
    
    # Build mapping: segment_id -> (object_id, label)
    segment_to_instance = {}
    seg_groups = data.get("segGroups", [])
    
    for group in seg_groups:
        object_id = group.get("objectId")
        label = group.get("label", "unknown")
        segments = group.get("segments", [])
        
        for seg_id in segments:
            segment_to_instance[seg_id] = (object_id, label)
    
    print(f"  ✓ {len(seg_groups)} object groups, {len(segment_to_instance)} segments")
    return segment_to_instance


def read_segs_json(segs_path: Path) -> dict:
    """
    Reads the *.segs.json file.
    Returns a dict with vertex-to-segment mapping.
    """
    print(f"[Read] Segmentation: {segs_path}")
    with open(segs_path, "r") as f:
        data = json.load(f)
    
    seg_indices = data.get("segIndices", [])
    print(f"  ✓ {len(seg_indices)} vertex-to-segment mappings")
    return {"segIndices": seg_indices}


def read_ply_binary(ply_path: Path):
    """
    Reads a binary PLY file (ScanNet format).
    Returns vertices as numpy array: (N, 7) -> x,y,z,r,g,b,alpha
    """
    print(f"[Read] PLY: {ply_path}")
    
    with open(ply_path, "rb") as f:
        # Read header
        header_lines = []
        while True:
            line = f.readline().decode("ascii").strip()
            header_lines.append(line)
            if line == "end_header":
                break
        
        # Parse header for vertex count
        num_vertices = 0
        num_faces = 0
        vertex_properties = []
        
        for line in header_lines:
            if line.startswith("element vertex"):
                num_vertices = int(line.split()[2])
            elif line.startswith("element face"):
                num_faces = int(line.split()[2])
            elif line.startswith("property"):
                parts = line.split()
                if len(parts) >= 3:
                    vertex_properties.append((parts[1], parts[2]))
        
        # Read vertex data (binary little endian)
        # Expected format: x,y,z (float), r,g,b,alpha (uchar)
        vertices = []
        for i in range(num_vertices):
            # Read floats (x, y, z)
            x, y, z = struct.unpack('<fff', f.read(12))
            # Read uchars (r, g, b, alpha)
            r, g, b, alpha = struct.unpack('<BBBB', f.read(4))
            vertices.append([x, y, z, r, g, b, alpha])
        
        # Skip face data (we don't need it for visualization)
        # Each face is: 1 uchar (num vertices) + N ints (vertex indices)
        # For simplicity, we'll just skip to the end
        
    vertices_array = np.array(vertices, dtype=float)
    print(f"  ✓ {len(vertices_array)} vertices")
    
    return {"vertices": vertices_array, "header_lines": header_lines}


def build_vertex_to_instance(segment_to_instance: dict, segs_data: dict):
    """
    Builds a mapping from vertex index to (object_id, label).
    Returns: dict[vertex_idx] -> (object_id, label)
    """
    seg_indices = segs_data.get("segIndices", [])
    vertex_to_instance = {}
    
    for vertex_idx, seg_id in enumerate(seg_indices):
        if seg_id in segment_to_instance:
            vertex_to_instance[vertex_idx] = segment_to_instance[seg_id]
        else:
            vertex_to_instance[vertex_idx] = (None, "unlabeled")
    
    return vertex_to_instance


def organize_by_instances(vertices: np.ndarray, vertex_to_instance: dict):
    """
    Groups vertices by instance_id (object_id) and assigns category from ground truth.
    vertices shape: (N, 7) -> x,y,z,r,g,b,alpha
    """
    inst = defaultdict(lambda: {
        "coords": [],
        "colors": [],
        "count": 0,
        "category": "unlabeled"
    })
    
    for vertex_idx, v in enumerate(vertices):
        x, y, z, r, g, b, alpha = v
        
        if vertex_idx in vertex_to_instance:
            object_id, label = vertex_to_instance[vertex_idx]
        else:
            object_id, label = (None, "unlabeled")
        
        # Use object_id as instance key (or -1 for unlabeled)
        instance_key = object_id if object_id is not None else -1
        
        inst[instance_key]["coords"].append([x, y, z])
        inst[instance_key]["colors"].append([r, g, b])
        inst[instance_key]["count"] += 1
        inst[instance_key]["category"] = label
    
    result = {}
    for instance_id, data in inst.items():
        coords = np.asarray(data["coords"], dtype=float)
        colors = np.asarray(data["colors"], dtype=float)
        result[int(instance_id)] = {
            "coords": coords,
            "colors": colors,
            "category": data["category"],
            "count": data["count"]
        }
    return result


def create_interactive_visualization(
    ply_data, 
    segment_to_instance: dict,
    segs_data: dict,
    output_html: Path, 
    show_unlabeled: bool = True
):
    """
    Builds and saves an interactive Plotly 3D visualization.
    - Unlabeled vertices (category == 'unlabeled') are shown in gray.
    - Labeled instances are colored by their average RGB.
    """
    vertices = ply_data["vertices"]
    
    print("[Info] Building vertex-to-instance mapping...")
    vertex_to_instance = build_vertex_to_instance(segment_to_instance, segs_data)
    
    print("[Info] Organizing by instances...")
    instances = organize_by_instances(vertices, vertex_to_instance)
    
    fig = go.Figure()
    
    unlabeled_instances = {k: v for k, v in instances.items() if v["category"] == "unlabeled"}
    labeled_instances = {k: v for k, v in instances.items() if v["category"] != "unlabeled"}
    
    print(f"  Unlabeled instances: {len(unlabeled_instances)}")
    print(f"  Labeled instances:   {len(labeled_instances)}")
    
    # Unlabeled vertices (gray)
    if show_unlabeled and unlabeled_instances:
        for inst_id, data in unlabeled_instances.items():
            coords = data["coords"]
            if coords.size == 0:
                continue
            fig.add_trace(go.Scatter3d(
                x=coords[:, 0], y=coords[:, 1], z=coords[:, 2],
                mode="markers",
                name=f"Unlabeled (ID:{inst_id})",
                marker=dict(size=2, color="rgb(120,120,120)", opacity=0.35),
                text=[f"Unlabeled<br>Instance: {inst_id}"] * len(coords),
                hovertemplate="<b>%{text}</b><br>X:%{x:.2f} Y:%{y:.2f} Z:%{z:.2f}<extra></extra>",
                showlegend=False
            ))
    
    # Labeled instances (average color)
    for inst_id, data in sorted(labeled_instances.items(), key=lambda x: x[1]["count"], reverse=True):
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
            text=f'ScanNet Ground Truth<br><sub>{total_vertices:,} vertices | {num_instances} instances</sub>',
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


def find_scene_files(script_dir: Path, scene_name: str) -> tuple[Path, Path, Path]:
    """
    Finds the required files for a scene.
    Returns: (ply_path, aggregation_path, segs_path)
    """
    scene_dir = script_dir / scene_name
    
    if not scene_dir.exists():
        raise FileNotFoundError(f"Scene directory not found: {scene_dir}")
    
    # Find files
    ply_files = list(scene_dir.glob("*.ply"))
    agg_files = list(scene_dir.glob("*.aggregation.json"))
    segs_files = list(scene_dir.glob("*.segs.json"))
    
    if not ply_files:
        raise FileNotFoundError(f"No PLY file found in {scene_dir}")
    if not agg_files:
        raise FileNotFoundError(f"No aggregation.json file found in {scene_dir}")
    if not segs_files:
        raise FileNotFoundError(f"No segs.json file found in {scene_dir}")
    
    return ply_files[0], agg_files[0], segs_files[0]


def output_html_path(script_dir: Path, scene_name: str, no_unlabeled: bool = False) -> Path:
    """Returns the output HTML path; saved inside the scene directory."""
    scene_dir = script_dir / scene_name
    suffix = "_nu" if no_unlabeled else ""
    return scene_dir / f"gt_map_3d_{scene_name}{suffix}.html"


def list_all_scene_names(script_dir: Path) -> list[str]:
    """Scans this folder for scene directories and returns their names."""
    scene_names = []
    for p in sorted(script_dir.iterdir()):
        if p.is_dir() and p.name.startswith("scene"):
            scene_names.append(p.name)
    return scene_names


def main():
    parser = argparse.ArgumentParser(
        description="Interactive 3D visualizer for ScanNet ground truth data"
    )
    parser.add_argument("-s", "--scene", type=str, default=None,
                        help='Scene name, e.g., "scene0000_01"')
    parser.add_argument("-u", "--no-unlabeled", action="store_true",
                        help='Hide unlabeled vertices (output filename ends with _nu.html)')
    
    args = parser.parse_args()
    script_dir = Path(__file__).parent.resolve()
    
    # Build worklist of scenes
    if args.scene:
        worklist = [args.scene]
    else:
        worklist = list_all_scene_names(script_dir)
        if not worklist:
            print("No scene directories found (expected names like 'scene0000_01').", file=sys.stderr)
            sys.exit(1)
    
    any_ok = False
    for scene_name in worklist:
        out_html = output_html_path(script_dir, scene_name, args.no_unlabeled)
        
        if out_html.exists():
            print(f"[Skip] Output already exists: {out_html}")
            any_ok = True
            continue
        
        try:
            ply_path, agg_path, segs_path = find_scene_files(script_dir, scene_name)
        except FileNotFoundError as e:
            print(f"[Skip] {e}", file=sys.stderr)
            continue
        
        try:
            # Read all data
            segment_to_instance = read_aggregation_json(agg_path)
            segs_data = read_segs_json(segs_path)
            ply_data = read_ply_binary(ply_path)
            
            # Create visualization
            create_interactive_visualization(
                ply_data=ply_data,
                segment_to_instance=segment_to_instance,
                segs_data=segs_data,
                output_html=out_html,
                show_unlabeled=not args.no_unlabeled
            )
            print(f"[OK] Scene {scene_name} -> {out_html}\n")
            any_ok = True
        except Exception as e:
            print(f"[Error] Scene {scene_name}: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc()
    
    if not any_ok:
        print("Nothing was processed. Check paths and file names.", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
