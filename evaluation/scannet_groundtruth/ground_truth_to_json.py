#!/usr/bin/env python3
import os
import re
import json
import numpy as np
import plyfile
import sys

def compute_gt_instances_aabb(scene_dir: str, scene_id: str) -> dict:
    """Compute GT instances AABB + class_name for a given scene directory."""
    ply_path  = os.path.join(scene_dir, f"{scene_id}_vh_clean_2.ply")
    segs_path = os.path.join(scene_dir, f"{scene_id}_vh_clean_2.0.010000.segs.json")
    aggr_path = os.path.join(scene_dir, f"{scene_id}.aggregation.json")

    # Basic existence checks
    missing = [p for p in (ply_path, segs_path, aggr_path) if not os.path.isfile(p)]
    if missing:
        raise FileNotFoundError(
            f"Missing required files for {scene_id}: " + ", ".join(missing)
        )

    # 1) vertices from PLY (mesh)
    ply = plyfile.PlyData.read(ply_path)
    V = np.vstack([
        ply['vertex'].data['x'],
        ply['vertex'].data['y'],
        ply['vertex'].data['z']
    ]).T  # (N,3)

    # 2) segIndices
    with open(segs_path, "r") as f:
        seg_json = json.load(f)
    segIndices = np.asarray(seg_json['segIndices'], dtype=np.int32)  # (N,)

    # 3) instances (aggregation)
    with open(aggr_path, "r") as f:
        aggr = json.load(f)
    instances = aggr['segGroups'] if 'segGroups' in aggr else aggr

    out = {"scene_id": scene_id, "instances": []}
    for inst_id, inst in enumerate(instances):
        segset = set(inst['segments'])
        vidx = np.nonzero(np.isin(segIndices, list(segset)))[0]
        if vidx.size == 0:
            continue
        pts = V[vidx]
        mins = pts.min(axis=0)
        maxs = pts.max(axis=0)
        out["instances"].append({
            "instance_id": int(inst_id),
            "class_name": inst["label"],
            "aabb": {
                "x_min": float(mins[0]), "y_min": float(mins[1]), "z_min": float(mins[2]),
                "x_max": float(maxs[0]), "y_max": float(maxs[1]), "z_max": float(maxs[2])
            }
        })
    return out


def main():
    # Directory where this script lives
    current_dir = os.path.dirname(os.path.abspath(__file__))

    # Pattern for scene directories: scene####_##
    pattern = re.compile(r"^scene\d{4}_\d{2}$")

    entries = sorted(os.listdir(current_dir))
    if not entries:
        print("No entries found in script directory.")
        sys.exit(0)

    any_processed = False

    for name in entries:
        scene_dir = os.path.join(current_dir, name)
        if not (os.path.isdir(scene_dir) and pattern.match(name)):
            continue  # skip non-scene dirs

        scene_id = name  # e.g., "scene0000_00"

        # JSON path **inside the scene directory**
        per_scene_json = os.path.join(scene_dir, f"{scene_id}_gt_instances_aabb.json")

        if os.path.isfile(per_scene_json):
            print(f"[Skip] {scene_id}: '{scene_id}_gt_instances_aabb.json' already present in scene dir.")
            continue

        print(f"[Process] {scene_id}: generating GT instances AABB...")

        try:
            out = compute_gt_instances_aabb(scene_dir, scene_id)
        except Exception as e:
            print(f"[Warn] {scene_id}: could not compute AABB ({e}). Skipping.")
            continue

        # Save result **inside the scene directory**
        with open(per_scene_json, "w") as f:
            json.dump(out, f, indent=2)

        print(f"[Done] Saved: {per_scene_json}")
        any_processed = True

    if not any_processed:
        print("Nothing to do: all scene directories already contain their *_gt_instances_aabb.json, or no scene dirs found.")


if __name__ == "__main__":
    main()
