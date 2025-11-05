#!/usr/bin/env python3
import os
import json
import sys


def compute_gt_instances_aabb(scene_dir: str, scene_id: str) -> dict:
    """
    Convert SceneNN semantic_ground_truth.json (center+size bbox format)
    to the same format used by ScanNet (min+max bbox format).
    
    Args:
        scene_dir: Directory containing the semantic_ground_truth.json file
        scene_id: Scene identifier (e.g., "011", "016", etc.)
    
    Returns:
        Dictionary with scene_id and instances in ScanNet format
    """
    semantic_gt_path = os.path.join(scene_dir, "semantic_ground_truth.json")
    
    if not os.path.isfile(semantic_gt_path):
        raise FileNotFoundError(f"Missing semantic_ground_truth.json for scene {scene_id}")
    
    # Load the semantic ground truth
    with open(semantic_gt_path, "r") as f:
        semantic_data = json.load(f)
    
    # Extract instances
    instances_dict = semantic_data.get("instances", {})
    
    # Convert to final format
    out = {"scene_id": scene_id, "instances": []}
    
    for obj_key in sorted(instances_dict.keys()):  # obj0, obj1, obj2, ...
        obj = instances_dict[obj_key]
        
        # Extract instance ID from obj_key (e.g., "obj0" -> 0)
        instance_id = int(obj_key.replace("obj", ""))
        
        # Get bounding box (center + size format)
        bbox = obj.get("bbox", {})
        center = bbox.get("center", [0, 0, 0])
        size = bbox.get("size", [0, 0, 0])
        
        # Get class name (first key in results dict, usually there's only one with value 1.0)
        results = obj.get("results", {})
        if not results:
            print(f"[Warn] {scene_id}/{obj_key}: no results/class_name found, skipping.")
            continue
        
        # Get the class name (key with highest confidence, usually 1.0)
        class_name = max(results.items(), key=lambda x: x[1])[0]
        
        # Convert center+size to min+max format
        # center = [cx, cy, cz], size = [sx, sy, sz]
        # min = center - size/2, max = center + size/2
        x_min = center[0] - size[0] / 2.0
        x_max = center[0] + size[0] / 2.0
        y_min = center[1] - size[1] / 2.0
        y_max = center[1] + size[1] / 2.0
        z_min = center[2] - size[2] / 2.0
        z_max = center[2] + size[2] / 2.0
        
        out["instances"].append({
            "instance_id": instance_id,
            "class_name": class_name,
            "aabb": {
                "x_min": float(x_min),
                "y_min": float(y_min),
                "z_min": float(z_min),
                "x_max": float(x_max),
                "y_max": float(y_max),
                "z_max": float(z_max)
            }
        })
    
    return out


def main():
    # Directory where this script lives
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Get all directories in scenenn_groundtruth
    entries = sorted(os.listdir(current_dir))
    if not entries:
        print("No entries found in script directory.")
        sys.exit(0)
    
    any_processed = False
    
    for name in entries:
        scene_dir = os.path.join(current_dir, name)
        
        # Skip files and check if it's a valid scene directory (numeric folder name)
        if not os.path.isdir(scene_dir) or not name.isdigit():
            continue
        
        scene_id = name  # e.g., "011", "016", etc.
        
        # Output JSON path inside the scene directory
        output_json = os.path.join(scene_dir, f"{scene_id}_gt_instances_aabb.json")
        
        if os.path.isfile(output_json):
            print(f"[Skip] {scene_id}: '{scene_id}_gt_instances_aabb.json' already exists.")
            continue
        
        print(f"[Process] {scene_id}: generating GT instances AABB...")
        
        try:
            out = compute_gt_instances_aabb(scene_dir, scene_id)
        except Exception as e:
            print(f"[Warn] {scene_id}: could not compute AABB ({e}). Skipping.")
            continue
        
        # Save result inside the scene directory
        with open(output_json, "w") as f:
            json.dump(out, f, indent=2)
        
        print(f"[Done] Saved: {output_json}")
        any_processed = True
    
    if not any_processed:
        print("Nothing to do: all scene directories already contain their *_gt_instances_aabb.json, "
              "or no valid scene directories found.")


if __name__ == "__main__":
    main()
