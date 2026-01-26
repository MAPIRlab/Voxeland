#!/usr/bin/env python3
"""
Extract all unique category labels from ScanNet ground truth aggregation files.

This script scans all scene directories in the current folder, reads their
*.aggregation.json files, and collects all unique 'label' values from the
'segGroups' lists. The result is saved as gt_categories.json.

Usage:
    python3 extract_gt_categories.py
"""

import json
import sys
from pathlib import Path


def main():
    script_dir = Path(__file__).parent.resolve()
    print(f"Scanning directory: {script_dir}")
    print("=" * 60)

    # Collect all unique labels
    unique_labels = set()
    scene_count = 0
    total_instances = 0

    # Iterate over all subdirectories (scenes)
    for scene_dir in sorted(script_dir.iterdir()):
        if not scene_dir.is_dir():
            continue

        # Look for aggregation.json files
        aggregation_files = list(scene_dir.glob("*.aggregation.json"))
        
        if not aggregation_files:
            continue

        scene_count += 1
        scene_name = scene_dir.name
        
        for agg_file in aggregation_files:
            print(f"[{scene_name}] Processing: {agg_file.name}")
            
            try:
                with open(agg_file, 'r') as f:
                    data = json.load(f)
                
                seg_groups = data.get('segGroups', [])
                scene_instances = 0
                
                for group in seg_groups:
                    label = group.get('label')
                    if label:
                        unique_labels.add(label)
                        scene_instances += 1
                        total_instances += 1
                
                print(f"  ✓ Found {scene_instances} instances in this file")
                
            except Exception as e:
                print(f"  ✗ Error reading {agg_file.name}: {e}", file=sys.stderr)
                continue

    # Sort labels alphabetically
    sorted_labels = sorted(unique_labels)

    # Save to JSON
    output_file = script_dir / "gt_categories.json"
    with open(output_file, 'w') as f:
        json.dump(sorted_labels, f, indent=2)

    print("=" * 60)
    print(f"Summary:")
    print(f"  - Scenes processed: {scene_count}")
    print(f"  - Total instances: {total_instances}")
    print(f"  - Unique categories: {len(sorted_labels)}")
    print(f"  - Output file: {output_file}")
    print("=" * 60)
    print("\nUnique categories found:")
    for i, label in enumerate(sorted_labels, 1):
        print(f"  {i:3d}. {label}")


if __name__ == "__main__":
    main()
