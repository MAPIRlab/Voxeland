#!/usr/bin/env python3
import os
import json
import sys
import numpy as np
import xml.etree.ElementTree as ET
from plyfile import PlyData


# SceneNN synonym mapping (from eval.py)
SCENENN_SYNONYMS = {
    "refridgerator": ["refrigerator", "fridge"],
    "refrigerator": ["refridgerator", "fridge"],
    "fridge": ["refrigerator", "refridgerator"],
    "TV": ["television"],
    "television": ["TV"],
    "desk": ["table"],
    "table": ["desk"],
    "book": ["books"],
    "books": ["book"]
}


def compute_gt_instances_aabb(scene_dir: str, scene_id: str) -> dict:
    """
    Compute GT instances AABB + class_name + synonyms for SceneNN scene.
    Reads from PLY (point cloud with label field) and XML (label->class mapping).
    
    Args:
        scene_dir: Directory containing {scene_id}.ply and {scene_id}.xml
        scene_id: Scene identifier (e.g., "011", "016", etc.)
    
    Returns:
        Dictionary with scene_id and instances with AABB and synonyms
    """
    ply_path = os.path.join(scene_dir, f"{scene_id}.ply")
    xml_path = os.path.join(scene_dir, f"{scene_id}.xml")
    
    # Check existence
    missing = [p for p in (ply_path, xml_path) if not os.path.isfile(p)]
    if missing:
        raise FileNotFoundError(f"Missing required files for {scene_id}: " + ", ".join(missing))
    
    # 1) Load XML to get label_id -> class_name mapping
    tree = ET.parse(xml_path)
    root = tree.getroot()
    
    label_to_class = {}
    for label_elem in root.findall('label'):
        label_id = label_elem.get('id')
        class_name = label_elem.get('text', '').strip()
        
        # Skip labels without valid ID or class name
        if not label_id or not class_name:
            continue
        
        label_to_class[int(label_id)] = class_name
    
    if not label_to_class:
        raise ValueError(f"No valid labels found in XML for scene {scene_id}")
    
    # 2) Load PLY to get vertices and their labels
    ply = PlyData.read(ply_path)
    vertex_data = ply['vertex'].data
    
    # Extract coordinates
    x = np.array(vertex_data['x'])
    y = np.array(vertex_data['y'])
    z = np.array(vertex_data['z'])
    labels = np.array(vertex_data['label'], dtype=np.int32)
    
    # Stack into (N, 3) array
    vertices = np.column_stack([x, y, z])
    
    # 3) For each unique label, compute AABB
    out = {"scene_id": scene_id, "instances": []}
    
    unique_labels = np.unique(labels)
    instance_id = 0
    
    for label_id in unique_labels:
        # Skip if label not in XML mapping
        if label_id not in label_to_class:
            continue
        
        # Get class name from XML
        class_name = label_to_class[label_id]
        
        # Remove trailing numbers from class name (e.g., "bed01" -> "bed", "pillow04" -> "pillow")
        import re
        class_name = re.sub(r'\d+$', '', class_name)
        
        # Get vertices for this label
        mask = labels == label_id
        instance_vertices = vertices[mask]
        
        if instance_vertices.shape[0] == 0:
            continue
        
        # Compute AABB
        mins = instance_vertices.min(axis=0)
        maxs = instance_vertices.max(axis=0)
        
        # Get synonyms for this class
        synonyms = SCENENN_SYNONYMS.get(class_name, [])
        
        out["instances"].append({
            "instance_id": instance_id,
            "class_name": class_name,
            "synonyms": synonyms,
            "aabb": {
                "x_min": float(mins[0]),
                "y_min": float(mins[1]),
                "z_min": float(mins[2]),
                "x_max": float(maxs[0]),
                "y_max": float(maxs[1]),
                "z_max": float(maxs[2])
            }
        })
        
        instance_id += 1
    
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
        output_json = os.path.join(scene_dir, f"{scene_id}_gt_instances_aabb_synonyms.json")
        
        if os.path.isfile(output_json):
            print(f"[Skip] {scene_id}: '{scene_id}_gt_instances_aabb_synonyms.json' already exists.")
            continue
        
        print(f"[Process] {scene_id}: generating GT instances AABB + synonyms from PLY and XML...")
        
        try:
            out = compute_gt_instances_aabb(scene_dir, scene_id)
        except Exception as e:
            print(f"[Warn] {scene_id}: could not compute AABB ({e}). Skipping.")
            import traceback
            traceback.print_exc()
            continue
        
        # Save result inside the scene directory
        with open(output_json, "w") as f:
            json.dump(out, f, indent=2)
        
        print(f"[Done] Saved: {output_json} ({len(out['instances'])} instances)")
        any_processed = True
    
    if not any_processed:
        print("Nothing to do: all scene directories already contain their *_gt_instances_aabb_synonyms.json, "
              "or no valid scene directories found.")


if __name__ == "__main__":
    main()
