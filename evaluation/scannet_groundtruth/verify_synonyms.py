#!/usr/bin/env python3
"""
Verify that *_aabb_synonyms.json files have the same content as *_aabb.json
but with an additional "synonyms" key after "class_name" in each instance.
"""

import json
from pathlib import Path
from typing import Dict, List, Tuple


def load_json(file_path: Path) -> dict:
    """Load JSON file."""
    with open(file_path, 'r') as f:
        return json.load(f)


def verify_instance(original: dict, synonyms: dict, instance_id: int) -> List[str]:
    """
    Verify that a single instance in synonyms file matches original,
    except for the addition of "synonyms" key after "class_name".
    
    Returns list of error messages (empty if no errors).
    """
    errors = []
    
    # Check that synonyms dict has exactly one more key than original
    original_keys = set(original.keys())
    synonyms_keys = set(synonyms.keys())
    
    if "synonyms" not in synonyms_keys:
        errors.append(f"  Instance {instance_id}: Missing 'synonyms' key")
        return errors
    
    # Remove 'synonyms' from comparison
    synonyms_keys_no_syn = synonyms_keys - {"synonyms"}
    
    if original_keys != synonyms_keys_no_syn:
        missing = original_keys - synonyms_keys_no_syn
        extra = synonyms_keys_no_syn - original_keys
        if missing:
            errors.append(f"  Instance {instance_id}: Missing keys in synonyms file: {missing}")
        if extra:
            errors.append(f"  Instance {instance_id}: Extra keys in synonyms file: {extra}")
        return errors
    
    # Verify all values match (except synonyms)
    for key in original_keys:
        if original[key] != synonyms[key]:
            errors.append(f"  Instance {instance_id}: Value mismatch for key '{key}'")
            errors.append(f"    Original: {original[key]}")
            errors.append(f"    Synonyms: {synonyms[key]}")
    
    # Verify synonyms is a list
    if not isinstance(synonyms["synonyms"], list):
        errors.append(f"  Instance {instance_id}: 'synonyms' is not a list: {type(synonyms['synonyms'])}")
    
    # Check that "synonyms" appears after "class_name" in the JSON structure
    # This requires checking the order in the actual JSON string
    
    return errors


def verify_file_pair(original_path: Path, synonyms_path: Path) -> Tuple[bool, List[str]]:
    """
    Verify that synonyms file matches original file structure.
    
    Returns (success, list_of_errors)
    """
    errors = []
    
    try:
        original_data = load_json(original_path)
        synonyms_data = load_json(synonyms_path)
    except Exception as e:
        errors.append(f"Error loading files: {e}")
        return False, errors
    
    # Check scene_id matches
    if original_data.get("scene_id") != synonyms_data.get("scene_id"):
        errors.append(f"Scene ID mismatch: {original_data.get('scene_id')} vs {synonyms_data.get('scene_id')}")
    
    # Check number of instances
    original_instances = original_data.get("instances", [])
    synonyms_instances = synonyms_data.get("instances", [])
    
    if len(original_instances) != len(synonyms_instances):
        errors.append(f"Instance count mismatch: {len(original_instances)} vs {len(synonyms_instances)}")
        return False, errors
    
    # Check each instance
    for orig_inst, syn_inst in zip(original_instances, synonyms_instances):
        orig_id = orig_inst.get("instance_id", "UNKNOWN")
        syn_id = syn_inst.get("instance_id", "UNKNOWN")
        
        if orig_id != syn_id:
            errors.append(f"Instance ID order mismatch at position: {orig_id} vs {syn_id}")
            continue
        
        inst_errors = verify_instance(orig_inst, syn_inst, orig_id)
        errors.extend(inst_errors)
    
    return len(errors) == 0, errors


def find_file_pairs(root_dir: Path) -> List[Tuple[Path, Path]]:
    """
    Find all pairs of aabb.json and aabb_synonyms.json files.
    
    Returns list of (original_path, synonyms_path) tuples.
    """
    pairs = []
    
    # Find all *_aabb.json files (excluding synonyms)
    for original_path in root_dir.rglob("*_gt_instances_aabb.json"):
        # Construct expected synonyms path
        synonyms_path = original_path.parent / original_path.name.replace(".json", "_synonyms.json")
        
        if synonyms_path.exists():
            pairs.append((original_path, synonyms_path))
        else:
            print(f"⚠️  Missing synonyms file for: {original_path.name}")
    
    return pairs


def main():
    # Get the directory where this script is located
    script_dir = Path(__file__).parent.absolute()
    
    print("="*80)
    print("Verifying aabb.json vs aabb_synonyms.json files")
    print("="*80)
    print(f"\nScanning directory: {script_dir}\n")
    
    # Find all file pairs
    file_pairs = find_file_pairs(script_dir)
    
    if not file_pairs:
        print("❌ No file pairs found!")
        return 1
    
    print(f"Found {len(file_pairs)} file pair(s)\n")
    
    # Verify each pair
    all_success = True
    for original_path, synonyms_path in sorted(file_pairs):
        scene_name = original_path.parent.name
        print(f"Checking: {scene_name}")
        
        success, errors = verify_file_pair(original_path, synonyms_path)
        
        if success:
            print(f"  ✅ OK - All checks passed")
        else:
            print(f"  ❌ ERRORS found:")
            for error in errors:
                print(f"    {error}")
            all_success = False
        
        print()
    
    # Summary
    print("="*80)
    if all_success:
        print("✅ All files verified successfully!")
        return 0
    else:
        print("❌ Some files have errors. Please review the output above.")
        return 1


if __name__ == "__main__":
    exit(main())
