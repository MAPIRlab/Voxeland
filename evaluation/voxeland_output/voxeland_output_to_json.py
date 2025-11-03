#!/usr/bin/env python3
"""
Convert Voxeland PLY semantic maps to JSON format matching ground truth structure.

This script processes all .ply files in subdirectories of the current directory,
extracts instance information and computes AABB for each instance, then saves
the result as JSON files with the same name (but .json extension).
"""

import os
import json
import numpy as np
from pathlib import Path


def read_ply_file(ply_path: str) -> dict:
    """
    Read a Voxeland PLY file and extract vertex data.
    
    Returns a dict with:
        - vertices: numpy array (N, 3) with x, y, z coordinates
        - instance_ids: numpy array (N,) with instance IDs
        - semantic_categories: list of strings (N,) with semantic categories
    """
    vertices = []
    instance_ids = []
    semantic_categories = []
    
    with open(ply_path, 'r') as f:
        # Skip header lines until we reach 'end_header'
        line = f.readline()
        while line.strip() != 'end_header':
            line = f.readline()
        
        # Read vertex data
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 8:  # x, y, z, r, g, b, instance_id, semantic_category
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                # r, g, b = int(parts[3]), int(parts[4]), int(parts[5])
                instance_id = int(parts[6])
                # Category can have multiple words (e.g., "washing machine")
                # Join all remaining parts from index 7 onwards
                semantic_category = ' '.join(parts[7:])
                
                vertices.append([x, y, z])
                instance_ids.append(instance_id)
                semantic_categories.append(semantic_category)
    
    return {
        'vertices': np.array(vertices),
        'instance_ids': np.array(instance_ids),
        'semantic_categories': semantic_categories
    }


def compute_instances_aabb(ply_data: dict, scene_id: str) -> dict:
    """
    Compute AABB for each instance in the PLY data.
    
    Args:
        ply_data: Dictionary with vertices, instance_ids, and semantic_categories
        scene_id: Scene identifier (e.g., "scene0000_01")
    
    Returns:
        Dictionary in the ground truth JSON format with scene_id and instances
    """
    vertices = ply_data['vertices']
    instance_ids = ply_data['instance_ids']
    semantic_categories = ply_data['semantic_categories']
    
    # Get unique instances
    unique_instances = np.unique(instance_ids)
    
    instances_list = []
    
    for inst_id in unique_instances:
        # Skip instance_id 0 (unknown) if desired, or include it
        # For now, we include all instances
        
        # Get all vertices for this instance
        mask = instance_ids == inst_id
        inst_vertices = vertices[mask]
        
        if inst_vertices.shape[0] == 0:
            continue
        
        # Get the semantic category (should be the same for all vertices of this instance)
        inst_categories = [semantic_categories[i] for i in range(len(semantic_categories)) if mask[i]]
        # Take the most common category (in case there are variations)
        class_name = max(set(inst_categories), key=inst_categories.count)
        
        # Compute AABB
        mins = inst_vertices.min(axis=0)
        maxs = inst_vertices.max(axis=0)
        
        instances_list.append({
            "instance_id": int(inst_id),
            "class_name": class_name,
            "aabb": {
                "x_min": float(mins[0]),
                "y_min": float(mins[1]),
                "z_min": float(mins[2]),
                "x_max": float(maxs[0]),
                "y_max": float(maxs[1]),
                "z_max": float(maxs[2])
            }
        })
    
    return {
        "scene_id": scene_id,
        "instances": instances_list
    }


def convert_ply_to_json(ply_path: str, json_path: str, scene_id: str):
    """
    Convert a single PLY file to JSON format.
    
    Args:
        ply_path: Path to input PLY file
        json_path: Path to output JSON file
        scene_id: Scene identifier
    """
    print(f"[Process] Converting: {ply_path}")
    
    try:
        # Read PLY file
        ply_data = read_ply_file(ply_path)
        
        # Compute instances and AABB
        result = compute_instances_aabb(ply_data, scene_id)
        
        # Save to JSON
        with open(json_path, 'w') as f:
            json.dump(result, f, indent=2)
        
        print(f"[Done] Saved: {json_path}")
        print(f"       Found {len(result['instances'])} instances")
        
    except Exception as e:
        print(f"[Error] Failed to convert {ply_path}: {e}")


def main():
    # Get the directory where this script is located
    current_dir = Path(__file__).parent.absolute()
    
    print(f"Scanning directory: {current_dir}")
    print("=" * 80)
    
    # Find all PLY files in subdirectories
    ply_files = list(current_dir.glob("**/*.ply"))
    
    if not ply_files:
        print("No PLY files found in subdirectories.")
        return
    
    print(f"Found {len(ply_files)} PLY file(s)\n")
    
    processed_count = 0
    skipped_count = 0
    
    for ply_path in sorted(ply_files):
        # Generate JSON path (same location and name, different extension)
        json_path = ply_path.with_suffix('.json')
        
        # Check if JSON already exists
        if json_path.exists():
            print(f"[Skip] {ply_path.name}: JSON already exists")
            skipped_count += 1
            continue
        
        # Extract scene_id from directory name
        scene_dir_name = ply_path.parent.name
        
        # Convert PLY to JSON
        convert_ply_to_json(str(ply_path), str(json_path), scene_dir_name)
        processed_count += 1
        print()
    
    print("=" * 80)
    print(f"Summary: {processed_count} file(s) converted, {skipped_count} file(s) skipped")


if __name__ == "__main__":
    main()
