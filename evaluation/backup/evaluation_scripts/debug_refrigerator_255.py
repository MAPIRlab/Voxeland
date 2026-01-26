#!/usr/bin/env python3
"""
Debug script to trace why refrigerator in scene 255 gets mAP=0
"""

import json
import sys
from pathlib import Path
import numpy as np
import open3d as o3d
from plyfile import PlyData

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))

from evaluation_jl_paper_categories import (
    normalize_class_name,
    read_voxeland_ply,
    load_scenenn_gt_points,
    compute_iou,
    compute_iou_v2,
    rotation_matrix,
    extract_scores_from_json,
    PAPER_CATEGORIES
)

def main():
    # Paths
    script_dir = Path(__file__).parent
    gt_dir = script_dir / 'scenenn_groundtruth' / '255'
    pred_dir = script_dir / 'voxeland_output' / 'scenenn_255'
    
    gt_json_path = gt_dir / '255_gt_instances_aabb_synonyms.json'
    gt_ply_path = gt_dir / '255.ply'
    gt_xml_path = gt_dir / '255.xml'
    
    pred_json_path = pred_dir / 'voxeland_semantic_map_detectron_scenenn_255.json'
    pred_ply_path = pred_dir / 'voxeland_semantic_map_detectron_scenenn_255.ply'
    
    print("="*80)
    print("DEBUGGING: Refrigerator in Scene 255")
    print("="*80)
    
    # Load GT JSON
    with open(gt_json_path, 'r') as f:
        gt_data = json.load(f)
    
    # Find refrigerator in GT
    gt_refrigerator = None
    for inst in gt_data['instances']:
        if inst['instance_id'] == 3:
            gt_refrigerator = inst
            break
    
    if gt_refrigerator:
        print("\n1. GT REFRIGERATOR (instance_id=3):")
        print(f"   Class: {gt_refrigerator['class_name']}")
        print(f"   Synonyms: {gt_refrigerator['synonyms']}")
        print(f"   AABB: {gt_refrigerator['aabb']}")
        
        # Normalize class name
        normalized_gt_class = normalize_class_name(gt_refrigerator['class_name'])
        print(f"   Normalized class: {normalized_gt_class}")
        print(f"   Is paper category: {normalized_gt_class in PAPER_CATEGORIES if normalized_gt_class else False}")
    
    # Load GT from XML to see what the script actually uses
    print("\n2. GT FROM XML:")
    if gt_xml_path.exists():
        import xml.etree.ElementTree as ET
        tree = ET.parse(gt_xml_path)
        root = tree.getroot()
        
        for child in root:
            nyu_class = child.attrib.get("nyu_class")
            label_id = child.attrib.get("id")
            
            if label_id == "3":
                print(f"   Label ID: {label_id}")
                print(f"   NYU Class: {nyu_class}")
                
                normalized = normalize_class_name(nyu_class)
                print(f"   Normalized: {normalized}")
                print(f"   Is paper category: {normalized in PAPER_CATEGORIES if normalized else False}")
                
                # Load GT points
                if gt_ply_path.exists():
                    gt_points = load_scenenn_gt_points(gt_ply_path, 0, int(label_id))
                    print(f"   GT Points loaded: {gt_points.shape[0]}")
                    if gt_points.shape[0] > 0:
                        print(f"   GT Points bounds: min={gt_points.min(axis=0)}, max={gt_points.max(axis=0)}")
    
    # Load prediction JSON
    print("\n3. PREDICTION JSON:")
    with open(pred_json_path, 'r') as f:
        pred_json = json.load(f)
    
    # Find refrigerator prediction (instance_id=37)
    pred_refrigerator = None
    for inst in pred_json['instances']:
        if inst['instance_id'] == 37:
            pred_refrigerator = inst
            break
    
    if pred_refrigerator:
        print(f"   Instance ID: {pred_refrigerator['instance_id']}")
        print(f"   Class: {pred_refrigerator['class_name']}")
        print(f"   AABB: {pred_refrigerator['aabb']}")
        print(f"   Scores: {pred_refrigerator.get('scores', {})}")
        
        # Extract and normalize scores
        categories, scores = extract_scores_from_json(pred_refrigerator)
        print(f"   Top category: {categories[0] if categories else 'None'}")
        print(f"   Top score: {scores[0] if scores else 0.0}")
        
        # Normalize prediction class
        normalized_pred_class = normalize_class_name(pred_refrigerator['class_name'])
        print(f"   Normalized class: {normalized_pred_class}")
        print(f"   Is paper category: {normalized_pred_class in PAPER_CATEGORIES if normalized_pred_class else False}")
    
    # Load prediction PLY and extract points
    print("\n4. PREDICTION POINT CLOUD:")
    if pred_ply_path.exists():
        pred_ply_data = read_voxeland_ply(str(pred_ply_path))
        
        # Extract points for instance 37
        pred_indexes = (pred_ply_data['instance_ids'] == 37)
        pred_points_raw = pred_ply_data['points'][pred_indexes]
        
        print(f"   Pred points (raw): {pred_points_raw.shape[0]}")
        if pred_points_raw.shape[0] > 0:
            print(f"   Pred points bounds (raw): min={pred_points_raw.min(axis=0)}, max={pred_points_raw.max(axis=0)}")
        
        # Apply axis transformation
        Rx = rotation_matrix(np.radians(90), (1, 0, 0))
        if pred_points_raw.shape[0] > 0:
            pred_points_transformed = np.linalg.inv(Rx)[:3,:3] @ pred_points_raw.T
            pred_points = pred_points_transformed.T
            print(f"   Pred points (transformed): {pred_points.shape[0]}")
            print(f"   Pred points bounds (transformed): min={pred_points.min(axis=0)}, max={pred_points.max(axis=0)}")
        else:
            pred_points = pred_points_raw
    
    # Compute IoU
    print("\n5. IoU COMPUTATION:")
    if gt_ply_path.exists() and pred_ply_path.exists():
        # Load GT points
        gt_points = load_scenenn_gt_points(gt_ply_path, 0, 3)
        
        if gt_points.shape[0] > 0 and pred_points.shape[0] > 0:
            iou1 = compute_iou(pred_points, gt_points, 0.02)
            iou2 = compute_iou_v2(pred_points, gt_points, 0.02)
            final_iou = max(iou1, iou2)
            
            print(f"   IoU (method 1): {iou1:.4f}")
            print(f"   IoU (method 2): {iou2:.4f}")
            print(f"   Final IoU (max): {final_iou:.4f}")
            print(f"   Threshold: 0.5")
            print(f"   Is TP? {final_iou >= 0.5}")
        else:
            print(f"   Cannot compute IoU: GT points={gt_points.shape[0]}, Pred points={pred_points.shape[0]}")
    
    # Check if instance is filtered out
    print("\n6. FILTERING ANALYSIS:")
    print("   Checking if prediction would be filtered out:")
    
    # Check if unknown
    is_unknown = pred_refrigerator['class_name'].lower() == 'unknown'
    print(f"   - Is 'unknown' class: {is_unknown}")
    
    # Check if has scores
    categories, scores = extract_scores_from_json(pred_refrigerator)
    has_scores = len(categories) > 0
    print(f"   - Has scores: {has_scores}")
    
    # Check if has AABB
    has_aabb = pred_refrigerator.get('aabb') is not None
    print(f"   - Has AABB: {has_aabb}")
    
    # Check if class is in paper categories
    normalized = normalize_class_name(pred_refrigerator['class_name'])
    is_paper_cat = normalized in PAPER_CATEGORIES if normalized else False
    print(f"   - Prediction class is paper category: {is_paper_cat}")
    
    print("\n7. MATCHING ANALYSIS:")
    print("   For a match to be TP, needs:")
    print("   1. IoU >= 0.5: ", end="")
    if gt_points.shape[0] > 0 and pred_points.shape[0] > 0:
        print(f"{final_iou >= 0.5} (IoU={final_iou:.4f})")
    else:
        print("Cannot compute")
    
    print("   2. Prediction not filtered (unknown, no scores, etc.): ", end="")
    not_filtered = not is_unknown and has_scores and has_aabb
    print(f"{not_filtered}")
    
    print("\n" + "="*80)
    print("CONCLUSION:")
    print("="*80)
    
    if normalized_gt_class is None:
        print("❌ GT refrigerator is NOT recognized as a paper category!")
        print(f"   GT class '{gt_refrigerator['class_name']}' normalized to '{normalized_gt_class}'")
        print(f"   This means it won't be included in the evaluation at all.")
    elif not is_paper_cat:
        print("❌ Prediction refrigerator is NOT recognized as a paper category!")
        print(f"   Pred class '{pred_refrigerator['class_name']}' normalized to '{normalized_pred_class}'")
    elif gt_points.shape[0] > 0 and pred_points.shape[0] > 0:
        if final_iou < 0.5:
            print(f"❌ IoU too low: {final_iou:.4f} < 0.5")
            print("   This is why mAP=0 - the prediction exists but IoU is below threshold")
        else:
            print(f"✅ Should be TP! IoU={final_iou:.4f} >= 0.5")
    else:
        print("❌ Missing point clouds - cannot compute IoU")
    
    print("="*80)

if __name__ == '__main__':
    main()
