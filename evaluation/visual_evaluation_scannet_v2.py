#!/usr/bin/env python3
"""
Visual Interactive Evaluation for ScanNet v2
Allows visual inspection of GT vs Prediction matches for debugging.

Usage:
    python3 visual_evaluation_scannet_v2.py -s 0000_01
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import numpy as np
import open3d as o3d

# Import evaluation functions from evaluation_scannet_v2
sys.path.insert(0, str(Path(__file__).parent))
from evaluation_scannet_v2 import (
    GT_DIR, PRED_DIR, VOXEL_SIZE, IOU_THRESHOLD,
    load_gt_aggregation, load_gt_segmentation, load_gt_ply_points,
    load_prediction_json, read_voxeland_ply, extract_instance_points,
    transform_prediction_points, compute_best_iou, class_matches,
    greedy_matching
)


def evaluate_scene_for_visualization(scene_id: str, detector: str) -> Optional[Dict]:
    """
    Evaluate a scene and return detailed matching information.
    
    Returns:
        Dict with gt_instances, pred_instances, and matches
    """
    # Construct paths
    gt_scene_dir = GT_DIR / scene_id
    pred_scene_dir = PRED_DIR / f'scannet_{scene_id}'
    
    # GT files
    aggregation_file = gt_scene_dir / f'{scene_id}.aggregation.json'
    segs_file = gt_scene_dir / f'{scene_id}_vh_clean_2.0.010000.segs.json'
    gt_ply_file = gt_scene_dir / f'{scene_id}_vh_clean_2.ply'
    
    # Prediction files
    pred_json_file = pred_scene_dir / f'voxeland_semantic_map_{detector}_scannet_{scene_id}.json'
    pred_ply_file = pred_scene_dir / f'voxeland_semantic_map_{detector}_scannet_{scene_id}.ply'
    
    # Check file existence
    required_files = [aggregation_file, segs_file, gt_ply_file, pred_json_file, pred_ply_file]
    for f in required_files:
        if not f.exists():
            print(f"[WARNING] Missing file: {f.name}")
            return None
    
    # Load GT data
    gt_aggregation = load_gt_aggregation(aggregation_file)
    seg_indices = load_gt_segmentation(segs_file)
    gt_all_points = load_gt_ply_points(gt_ply_file)
    
    # Load prediction data
    pred_json = load_prediction_json(pred_json_file)
    pred_ply_data = read_voxeland_ply(str(pred_ply_file))
    pred_all_points = pred_ply_data['points']
    pred_instance_ids = pred_ply_data['instance_ids']
    
    # Transform prediction points
    pred_all_points_transformed = transform_prediction_points(pred_all_points)
    
    # Build GT instances
    gt_instances = {}
    for obj_id, info in gt_aggregation.items():
        label = info['label']
        segments = info['segments']
        points = extract_instance_points(gt_all_points, seg_indices, segments)
        
        if points.shape[0] == 0:
            continue
        
        # Voxelize
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
        points = np.array(pcd.points)
        
        gt_instances[obj_id] = {
            'class': label,
            'points': points
        }
    
    # Build prediction instances
    pred_instances = {}
    for inst_key, inst_data in pred_json.items():
        inst_id = int(inst_key.replace('obj', ''))
        if inst_id == 0:
            continue
        
        scores = inst_data.get('results', {})
        valid_scores = {k.lower().strip(): v for k, v in scores.items() 
                       if k.lower().strip() != 'unknown' and isinstance(v, (int, float))}
        
        if not valid_scores:
            continue
        
        pred_class = max(valid_scores, key=valid_scores.get)
        total = sum(valid_scores.values())
        confidence = valid_scores[pred_class] / total if total > 0 else 0.0
        
        mask = pred_instance_ids == inst_id
        points = pred_all_points_transformed[mask]
        
        if points.shape[0] == 0:
            continue
        
        pred_instances[inst_id] = {
            'class': pred_class,
            'confidence': confidence,
            'points': points
        }
    
    # Perform matching
    matches = greedy_matching(gt_instances, pred_instances, IOU_THRESHOLD)
    
    return {
        'gt_instances': gt_instances,
        'pred_instances': pred_instances,
        'matches': matches
    }


def get_match_info(gt_id: int, evaluation_data: Dict) -> Tuple[Optional[int], float, bool, str]:
    """
    Get matching information for a GT instance.
    
    Returns:
        (pred_id, iou, class_match, status)
        status: 'TP', 'FP', 'NO_MATCH', 'NO_PREDICTIONS'
    """
    gt_instances = evaluation_data['gt_instances']
    pred_instances = evaluation_data['pred_instances']
    matches = evaluation_data['matches']
    
    if gt_id not in gt_instances:
        return None, 0.0, False, 'INVALID_GT_ID'
    
    gt_info = gt_instances[gt_id]
    
    # Check if matched
    for gt_m, pred_m, iou_m, class_m in matches:
        if gt_m == gt_id:
            return pred_m, iou_m, class_m, 'TP' if class_m else 'FP'
    
    # Not matched, find best candidate
    best_iou = 0.0
    best_pred_id = None
    
    for pred_id, pred_info in pred_instances.items():
        iou = compute_best_iou(gt_info['points'], pred_info['points'], VOXEL_SIZE)
        if iou > best_iou:
            best_iou = iou
            best_pred_id = pred_id
    
    if best_pred_id and best_iou > 0:
        return best_pred_id, best_iou, False, 'NO_MATCH'
    
    return None, 0.0, False, 'NO_PREDICTIONS'


def format_match_message(detector: str, gt_id: int, gt_class: str, 
                         pred_id: Optional[int], iou: float, class_match: bool,
                         status: str, pred_instances: Dict) -> str:
    """Format a human-readable message about the match."""
    
    if status == 'INVALID_GT_ID':
        return f"[{detector.upper()}] ERROR: Invalid GT ID"
    
    if status == 'NO_PREDICTIONS':
        return f"[{detector.upper()}] GT#{gt_id} '{gt_class}' -> NO PREDICTIONS AVAILABLE"
    
    pred_class = pred_instances[pred_id]['class']
    pred_conf = pred_instances[pred_id]['confidence']
    
    if status == 'TP':
        return f"[{detector.upper()}] GT#{gt_id} '{gt_class}' -> Pred#{pred_id} '{pred_class}' | IoU={iou:.3f} | ✓ TRUE POSITIVE (conf={pred_conf:.2f})"
    
    if status == 'FP':
        return f"[{detector.upper()}] GT#{gt_id} '{gt_class}' -> Pred#{pred_id} '{pred_class}' | IoU={iou:.3f} | ✗ FALSE POSITIVE - class mismatch (conf={pred_conf:.2f})"
    
    if status == 'NO_MATCH':
        return f"[{detector.upper()}] GT#{gt_id} '{gt_class}' -> Pred#{pred_id} '{pred_class}' | IoU={iou:.3f} | ✗ NO MATCH - IoU too low (threshold={IOU_THRESHOLD})"
    
    return f"[{detector.upper()}] Unknown status"


def visualize_gt_pred(gt_points: np.ndarray, pred_points: np.ndarray, 
                      gt_class: str, pred_class: str, gt_id: int, pred_id: int,
                      iou: float, detector: str):
    """
    Visualize GT and prediction point clouds in 3D.
    GT: Green, Pred: Red
    Z-axis (blue) is shown as vertical
    """
    # Create GT point cloud (green)
    gt_pcd = o3d.geometry.PointCloud()
    gt_pcd.points = o3d.utility.Vector3dVector(gt_points)
    gt_pcd.paint_uniform_color([0.0, 1.0, 0.0])  # Green
    
    # Create Pred point cloud (red)
    pred_pcd = o3d.geometry.PointCloud()
    pred_pcd.points = o3d.utility.Vector3dVector(pred_points)
    pred_pcd.paint_uniform_color([1.0, 0.0, 0.0])  # Red
    
    # Create coordinate frame
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
    
    # Create window title
    window_name = f"[{detector.upper()}] GT#{gt_id} '{gt_class}' (GREEN) vs Pred#{pred_id} '{pred_class}' (RED) | IoU={iou:.3f}"
    
    print(f"\n    Opening visualization window...")
    print(f"    Controls: Mouse to rotate, Scroll to zoom, Close window to continue")
    
    # Create visualizer with custom view (Z-axis up)
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=window_name, width=1200, height=800, left=100, top=100)
    
    # Add geometries
    vis.add_geometry(gt_pcd)
    vis.add_geometry(pred_pcd)
    vis.add_geometry(coord_frame)
    
    # Set view with Z-axis vertical
    view_control = vis.get_view_control()
    view_control.set_front([0, -1, 0])  # Looking from -Y towards +Y
    view_control.set_up([0, 0, 1])      # Z-axis is up (blue vertical)
    view_control.set_lookat(np.mean(gt_points, axis=0))  # Center on GT
    view_control.set_zoom(0.8)
    
    # Run visualizer
    vis.run()
    vis.destroy_window()


def interactive_evaluation(scene_id: str):
    """
    Interactive evaluation loop for a scene.
    """
    print("=" * 80)
    print(f"INTERACTIVE VISUAL EVALUATION - Scene: {scene_id}")
    print("=" * 80)
    print("\nLoading scene data and computing evaluation...")
    
    # Load evaluation data for all detectors
    detectors = ['detectron', 'talos', 'yoloe']
    evaluations = {}
    
    for detector in detectors:
        print(f"  Evaluating {detector}...", end=" ")
        eval_data = evaluate_scene_for_visualization(scene_id, detector)
        if eval_data:
            evaluations[detector] = eval_data
            print(f"✓ ({len(eval_data['gt_instances'])} GT, {len(eval_data['pred_instances'])} Pred)")
        else:
            print("✗ FAILED")
    
    if not evaluations:
        print("\nERROR: Could not load evaluation data for any detector!")
        return
    
    # Get GT instance count
    first_detector = list(evaluations.keys())[0]
    gt_instances = evaluations[first_detector]['gt_instances']
    n_gt = len(gt_instances)
    gt_ids = sorted(gt_instances.keys())
    
    print(f"\n✓ Scene loaded successfully!")
    print(f"  GT instances: {n_gt} (IDs: {gt_ids[0]} to {gt_ids[-1]})")
    print(f"  Available detectors: {', '.join(evaluations.keys())}")
    print("\n" + "=" * 80)
    
    # Interactive loop
    while True:
        print(f"\n{'─' * 80}")
        gt_id_str = input(f"Enter GT instance ID (0-{gt_ids[-1]}) or 'quit' to exit: ").strip()
        
        if gt_id_str.lower() in ['quit', 'q', 'exit']:
            print("\nExiting...")
            break
        
        if not gt_id_str:
            continue
        
        try:
            gt_id = int(gt_id_str)
        except ValueError:
            print("  ⚠ Invalid input. Please enter a number.")
            continue
        
        if gt_id not in gt_instances:
            print(f"  ⚠ GT ID {gt_id} not found. Available IDs: {gt_ids}")
            continue
        
        gt_class = gt_instances[gt_id]['class']
        gt_points_count = len(gt_instances[gt_id]['points'])
        
        print(f"\n  GT#{gt_id}: '{gt_class}' ({gt_points_count} points)")
        print(f"  {'─' * 76}")
        
        # Get match info for each detector
        match_infos = {}
        valid_detectors = []
        
        for detector in evaluations.keys():
            pred_id, iou, class_match, status = get_match_info(gt_id, evaluations[detector])
            match_infos[detector] = (pred_id, iou, class_match, status)
            
            message = format_match_message(
                detector, gt_id, gt_class, pred_id, iou, class_match, status,
                evaluations[detector]['pred_instances']
            )
            print(f"  {message}")
            
            if status != 'NO_PREDICTIONS':
                valid_detectors.append(detector)
        
        # If all are NO_PREDICTIONS, continue automatically
        if not valid_detectors:
            print(f"\n  ⚠ All detectors have NO PREDICTIONS AVAILABLE for this GT instance.")
            print(f"  Skipping visualization...")
            continue
        
        # Ask user which detector to visualize
        while True:
            print(f"\n  Valid detectors for visualization: {', '.join(valid_detectors)}")
            detector_choice = input(f"  Choose detector to visualize (or press Enter for new GT ID): ").strip().lower()
            
            if not detector_choice:
                break
            
            if detector_choice not in valid_detectors:
                print(f"  ⚠ Invalid choice. Choose from: {', '.join(valid_detectors)}")
                continue
            
            # Visualize
            pred_id, iou, class_match, status = match_infos[detector_choice]
            gt_points = gt_instances[gt_id]['points']
            pred_points = evaluations[detector_choice]['pred_instances'][pred_id]['points']
            pred_class = evaluations[detector_choice]['pred_instances'][pred_id]['class']
            
            visualize_gt_pred(
                gt_points, pred_points,
                gt_class, pred_class,
                gt_id, pred_id, iou,
                detector_choice
            )
            
            print(f"\n  Window closed. Choose another detector or press Enter to continue.")


def main():
    parser = argparse.ArgumentParser(
        description='Interactive visual evaluation of ScanNet v2 predictions'
    )
    parser.add_argument(
        '-s', '--scene',
        type=str,
        required=True,
        help='Scene ID (e.g., "0000_01", "0002_01")'
    )
    
    args = parser.parse_args()
    scene_id = f"scene{args.scene}"
    
    # Check if scene exists
    gt_scene_dir = GT_DIR / scene_id
    if not gt_scene_dir.exists():
        print(f"ERROR: Scene '{scene_id}' not found in {GT_DIR}")
        sys.exit(1)
    
    interactive_evaluation(scene_id)


if __name__ == '__main__':
    main()
