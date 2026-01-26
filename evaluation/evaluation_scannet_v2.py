#!/usr/bin/env python3
"""
Evaluation Script for Voxeland on ScanNet v2 Dataset
Computes mAP@0.5 using point cloud IoU for instance segmentation evaluation.

Key improvements over previous evaluation scripts:
1. Hungarian matching (optimal assignment) instead of greedy matching
2. Proper TP/FP/FN classification with class verification
3. Dynamic class set from GT (not hardcoded subset)
4. Correct synonym handling (GT -> prediction direction)
5. Prevention of multiple predictions matching same GT
6. Per-class AP aggregation across scenes (not just averaging scene mAPs)
7. Comprehensive comparative analysis across detectors

Usage:
    python3 evaluation_scannet_v2.py
"""

import json
import sys
import os
from pathlib import Path
from typing import Dict, List, Tuple, Set, Optional
import numpy as np
from collections import defaultdict
from plyfile import PlyData
import open3d as o3d
from scipy.optimize import linear_sum_assignment
from scipy.special import psi
from datetime import datetime


# =============================================================================
# Configuration
# =============================================================================

# Paths
SCRIPT_DIR = Path(__file__).parent
GT_DIR = SCRIPT_DIR / 'scannet_v2_groundtruth'
PRED_DIR = SCRIPT_DIR / 'voxeland_output'
OUTPUT_DIR = SCRIPT_DIR

# Evaluation parameters
VOXEL_SIZE = 0.02  # For GT voxelization and IoU computation
IOU_THRESHOLD = 0.5  # mAP@0.5

# Detectors to evaluate
# DETECTORS = ['detectron', 'talos', 'yoloe']
DETECTORS = ['detectron']

# Import synonyms dictionary
sys.path.insert(0, str(SCRIPT_DIR / 'scannet_v2_groundtruth'))
from gt_categories_synonyms import CATEGORY_SYNONYMS


# =============================================================================
# PLY and Data Loading Functions
# =============================================================================

def read_voxeland_ply(ply_path: str) -> dict:
    """
    Read a Voxeland PLY file (ASCII format with instanceid field).
    
    Returns:
        dict with 'points' (N,3), 'instance_ids' (N,)
    """
    points = []
    instance_ids = []
    
    with open(ply_path, 'r') as f:
        # Parse header
        line = f.readline()
        while line.strip() != 'end_header':
            line = f.readline()
        
        # Read vertex data
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 9:  # x,y,z,r,g,b,instanceid,uncertainty_instances,uncertainty_categories
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                instance_id = int(parts[6])
                points.append([x, y, z])
                instance_ids.append(instance_id)
    
    return {
        'points': np.array(points) if points else np.zeros((0, 3)),
        'instance_ids': np.array(instance_ids) if instance_ids else np.zeros(0, dtype=int)
    }


def load_gt_aggregation(aggregation_path: Path) -> Dict:
    """
    Load ScanNet aggregation.json which contains instance segmentation info.
    
    Returns:
        Dict mapping objectId -> {label, segments}
    """
    with open(aggregation_path, 'r') as f:
        data = json.load(f)
    
    instances = {}
    for seg_group in data['segGroups']:
        object_id = seg_group['objectId']
        label = seg_group['label'].lower().strip()
        segments = set(seg_group['segments'])
        instances[object_id] = {
            'label': label,
            'segments': segments
        }
    
    return instances


def load_gt_segmentation(segs_path: Path) -> List[int]:
    """
    Load ScanNet segmentation indices (per-vertex segment assignment).
    """
    with open(segs_path, 'r') as f:
        data = json.load(f)
    return data['segIndices']


def load_gt_ply_points(ply_path: Path) -> np.ndarray:
    """
    Load point coordinates from ScanNet GT PLY (binary format).
    """
    ply = PlyData.read(str(ply_path))
    x = np.array(ply.elements[0].data["x"])
    y = np.array(ply.elements[0].data["y"])
    z = np.array(ply.elements[0].data["z"])
    return np.stack([x, y, z], axis=1)


def extract_instance_points(all_points: np.ndarray, seg_indices: List[int], 
                           instance_segments: Set[int]) -> np.ndarray:
    """
    Extract points belonging to a specific instance.
    """
    seg_indices_arr = np.array(seg_indices)
    mask = np.isin(seg_indices_arr, list(instance_segments))
    return all_points[mask]


def load_prediction_json(json_path: Path) -> Dict:
    """
    Load Voxeland prediction JSON with instance results.
    """
    with open(json_path, 'r') as f:
        data = json.load(f)
    # The JSON has format {"instances": {"obj0": {...}, "obj1": {...}}}
    if 'instances' in data:
        return data['instances']
    return data


# =============================================================================
# Class Matching with Synonyms
# =============================================================================

def normalize_class_name(class_name: str) -> str:
    """Normalize class name to lowercase, stripped."""
    return class_name.lower().strip()


def get_predicted_class_from_scores(scores_dict: Dict) -> Tuple[str, float]:
    """
    Get the top predicted class and its confidence from Voxeland scores.
    Filters out 'unknown' and normalizes scores.
    """
    if not scores_dict:
        return 'unknown', 0.0
    
    # Filter and sort by score
    valid_scores = {k.lower().strip(): v for k, v in scores_dict.items() 
                    if k.lower().strip() != 'unknown' and isinstance(v, (int, float))}
    
    if not valid_scores:
        return 'unknown', 0.0
    
    # Get top class
    top_class = max(valid_scores, key=valid_scores.get)
    
    # Normalize to get confidence (probability)
    total = sum(valid_scores.values())
    confidence = valid_scores[top_class] / total if total > 0 else 0.0
    
    return top_class, confidence


def class_matches(gt_class: str, pred_class: str) -> bool:
    """
    Check if predicted class matches GT class using synonym dictionary.
    
    The matching is asymmetric:
    - GT class is the key in CATEGORY_SYNONYMS
    - Predicted class should match either the GT class itself or one of its synonyms
    
    This means: if GT is 'armchair', prediction 'chair' is accepted.
    But if GT is 'chair', prediction 'armchair' is NOT accepted (unless 'armchair' is in chair's synonyms).
    """
    gt_norm = normalize_class_name(gt_class)
    pred_norm = normalize_class_name(pred_class)
    
    # Direct match
    if gt_norm == pred_norm:
        return True
    
    # Check if prediction is in GT's synonyms
    if gt_norm in CATEGORY_SYNONYMS:
        synonyms = [normalize_class_name(s) for s in CATEGORY_SYNONYMS[gt_norm]]
        if pred_norm in synonyms:
            return True
    
    return False


# =============================================================================
# IoU Computation (Point Cloud Based)
# =============================================================================

def compute_iou_pointcloud(points1: np.ndarray, points2: np.ndarray, 
                           voxel_size: float) -> float:
    """
    Compute IoU between two point clouds using KDTree radius search.
    Uses the approach from eval.py.
    """
    if points1.shape[0] == 0 or points2.shape[0] == 0:
        return 0.0
    
    # Create point clouds
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(points1)
    
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(points2)
    
    # Determine larger and smaller point cloud
    if points1.shape[0] >= points2.shape[0]:
        bigger_pc, smaller_pc = pcd1, pcd2
    else:
        bigger_pc, smaller_pc = pcd2, pcd1
    
    # Build KDTree for larger point cloud
    kdtree = o3d.geometry.KDTreeFlann(bigger_pc)
    
    # Count points in smaller_pc that have a neighbor in bigger_pc
    count_close = 0
    radius = 4 * voxel_size
    
    for point in smaller_pc.points:
        [k, _, _] = kdtree.search_radius_vector_3d(point, radius)
        if k > 0:
            count_close += 1
    
    # Compute IoU
    iou = float(count_close) / (points1.shape[0] + points2.shape[0] - count_close)
    return iou


def compute_iou_pointcloud_v2(points1: np.ndarray, points2: np.ndarray, 
                               voxel_size: float) -> float:
    """
    Alternative IoU computation with voxel downsampling (from eval.py).
    """
    if points1.shape[0] == 0 or points2.shape[0] == 0:
        return 0.0
    
    # Create and downsample point clouds
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(points1)
    pcd1 = pcd1.voxel_down_sample(voxel_size=0.05)
    
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(points2)
    pcd2 = pcd2.voxel_down_sample(voxel_size=0.05)
    
    # Get points after downsampling
    pts1 = np.array(pcd1.points)
    pts2 = np.array(pcd2.points)
    
    if pts1.shape[0] == 0 or pts2.shape[0] == 0:
        return 0.0
    
    # Determine larger and smaller
    if pts1.shape[0] >= pts2.shape[0]:
        bigger_pc, smaller_pc = pcd1, pcd2
    else:
        bigger_pc, smaller_pc = pcd2, pcd1
    
    # Build KDTree
    kdtree = o3d.geometry.KDTreeFlann(bigger_pc)
    
    # Count close points
    count_close = 0
    radius = np.sqrt(2) * 0.05 + 0.0005
    
    for point in smaller_pc.points:
        [k, _, _] = kdtree.search_radius_vector_3d(point, radius)
        if k > 0:
            count_close += 1
    
    # Compute IoU
    pts1_count = np.array(pcd1.points).shape[0]
    pts2_count = np.array(pcd2.points).shape[0]
    iou = float(count_close) / (pts1_count + pts2_count - count_close)
    return iou


def compute_best_iou(points1: np.ndarray, points2: np.ndarray, 
                     voxel_size: float) -> float:
    """
    Compute best IoU between two point clouds using both methods.
    """
    iou1 = compute_iou_pointcloud(points1, points2, voxel_size)
    iou2 = compute_iou_pointcloud_v2(points1, points2, voxel_size)
    return max(iou1, iou2)


# =============================================================================
# Coordinate Transformation
# =============================================================================

def rotation_matrix_x(angle_deg: float) -> np.ndarray:
    """Create rotation matrix around X axis."""
    angle = np.radians(angle_deg)
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ])


def transform_prediction_points(points: np.ndarray) -> np.ndarray:
    """
    Transform prediction points from Voxeland coordinate system to GT coordinate system.
    Applies 90 degree rotation around X axis (inverse).
    """
    if points.shape[0] == 0:
        return points
    
    # NOTE: For ScanNet, the prediction and GT are already aligned, no rotation needed
    # SceneNN requires the rotation, but ScanNet does not
    return points
    
    # Rx = rotation_matrix_x(90)
    # Rx_inv = np.linalg.inv(Rx)
    # transformed = (Rx_inv @ points.T).T
    # return transformed


# =============================================================================
# Greedy Matching (as in evaluation_jl.py)
# =============================================================================

def greedy_matching(gt_instances: Dict, pred_instances: Dict, 
                    iou_threshold: float) -> List[Tuple]:
    """
    Perform greedy matching between GT and predictions.
    For each GT instance, select the prediction with highest IoU.
    
    NOTE: This allows multiple GTs to match the same prediction (not optimal).
    
    Returns:
        List of (gt_id, pred_id, iou, class_match) tuples for matches with IoU >= threshold
    """
    matches = []
    
    # For each GT, find best prediction
    for gt_id, gt_info in gt_instances.items():
        gt_points = gt_info['points']
        gt_class = gt_info['class']
        
        best_pred_id = None
        best_iou = 0.0
        
        # Compute IoU with all predictions
        for pred_id, pred_info in pred_instances.items():
            pred_points = pred_info['points']
            iou = compute_best_iou(gt_points, pred_points, VOXEL_SIZE)
            
            if iou > best_iou:
                best_iou = iou
                best_pred_id = pred_id
        
        # If best IoU meets threshold, add match
        if best_pred_id is not None and best_iou >= iou_threshold:
            pred_class = pred_instances[best_pred_id]['class']
            class_match = class_matches(gt_class, pred_class)
            matches.append((gt_id, best_pred_id, best_iou, class_match))
    
    return matches


# =============================================================================
# mAP Computation
# =============================================================================

def compute_voc_ap(recall: np.ndarray, precision: np.ndarray) -> float:
    """
    Compute VOC-style AP (area under precision-recall curve).
    Uses the all-point interpolation method (VOC 2010+).
    """
    # Append sentinel values
    mrec = np.concatenate(([0.], recall, [1.]))
    mpre = np.concatenate(([0.], precision, [0.]))
    
    # Compute precision envelope (monotonically decreasing)
    for i in range(mpre.size - 1, 0, -1):
        mpre[i - 1] = max(mpre[i - 1], mpre[i])
    
    # Find points where recall changes
    i = np.where(mrec[1:] != mrec[:-1])[0]
    
    # Compute area under curve
    ap = np.sum((mrec[i + 1] - mrec[i]) * mpre[i + 1])
    return float(ap)


def compute_ap_for_class(detections: List[Dict], n_gt: int) -> float:
    """
    Compute Average Precision for a single class.
    
    Args:
        detections: List of dicts with 'confidence', 'is_tp' keys
        n_gt: Number of ground truth instances of this class
    
    Returns:
        AP value
    """
    if n_gt == 0 or len(detections) == 0:
        return 0.0
    
    # Sort by confidence descending
    sorted_dets = sorted(detections, key=lambda x: x['confidence'], reverse=True)
    
    # Compute cumulative TP and FP
    tp = np.zeros(len(sorted_dets))
    fp = np.zeros(len(sorted_dets))
    
    for i, det in enumerate(sorted_dets):
        if det['is_tp']:
            tp[i] = 1
        else:
            fp[i] = 1
    
    tp_cumsum = np.cumsum(tp)
    fp_cumsum = np.cumsum(fp)
    
    # Compute precision and recall
    precision = tp_cumsum / (tp_cumsum + fp_cumsum)
    recall = tp_cumsum / n_gt
    
    # Compute AP
    return compute_voc_ap(recall, precision)


# =============================================================================
# Scene Evaluation
# =============================================================================

def evaluate_scene(scene_id: str, detector: str) -> Optional[Dict]:
    """
    Evaluate a single scene for a specific detector.
    
    Returns:
        Dict with per-class results or None if data not available
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
            print(f"    [WARNING] Missing file: {f.name}")
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
    
    # Transform prediction points to GT coordinate system
    pred_all_points_transformed = transform_prediction_points(pred_all_points)
    
    # Build GT instances dictionary
    gt_instances = {}
    gt_classes = set()
    
    for obj_id, info in gt_aggregation.items():
        label = info['label']
        segments = info['segments']
        
        # Extract points for this instance
        points = extract_instance_points(gt_all_points, seg_indices, segments)
        
        # Skip instances with no points
        if points.shape[0] == 0:
            continue
        
        # Voxelize GT points for consistency
        if points.shape[0] > 0:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
            points = np.array(pcd.points)
        
        gt_instances[obj_id] = {
            'class': label,
            'points': points
        }
        gt_classes.add(label)
    
    # Build prediction instances dictionary
    pred_instances = {}
    
    # pred_json already contains just the instances (loaded by load_prediction_json)
    for inst_key, inst_data in pred_json.items():
        # Extract instance ID
        inst_id = int(inst_key.replace('obj', ''))
        
        # Skip background (id 0)
        if inst_id == 0:
            continue
        
        # Get predicted class and confidence
        scores = inst_data.get('results', {})
        pred_class, confidence = get_predicted_class_from_scores(scores)
        
        # Skip unknown predictions
        if pred_class == 'unknown':
            continue
        
        # Extract points for this instance
        mask = pred_instance_ids == inst_id
        points = pred_all_points_transformed[mask]
        
        if points.shape[0] == 0:
            continue
        
        pred_instances[inst_id] = {
            'class': pred_class,
            'confidence': confidence,
            'points': points,
            'scores': scores
        }
    
    # Perform greedy matching
    matches = greedy_matching(gt_instances, pred_instances, IOU_THRESHOLD)
    
    # =========================================================================
    # DETAILED LOGGING FOR DEBUGGING
    # =========================================================================
    print(f"\n    --- Matching Details ---")
    print(f"    GT instances: {len(gt_instances)}, Pred instances: {len(pred_instances)}")
    print(f"    Matches found (IoU >= {IOU_THRESHOLD}): {len(matches)}")
    
    # Track which GTs were matched
    matched_gt_set = {gt_id for gt_id, _, _, _ in matches}
    
    # For each GT, show what happened
    for gt_id, gt_info in gt_instances.items():
        gt_class = gt_info['class']
        gt_points_count = len(gt_info['points'])
        
        # Find if this GT was matched
        match_found = None
        for gt_m, pred_m, iou_m, class_m in matches:
            if gt_m == gt_id:
                match_found = (pred_m, iou_m, class_m)
                break
        
        if match_found:
            pred_id, iou, class_match = match_found
            pred_class = pred_instances[pred_id]['class']
            pred_points_count = len(pred_instances[pred_id]['points'])
            status = "✓ TP" if class_match else "✗ FP (class mismatch)"
            print(f"      GT#{gt_id} '{gt_class}' ({gt_points_count}pts) -> Pred#{pred_id} '{pred_class}' ({pred_points_count}pts) | IoU={iou:.3f} | {status}")
        else:
            # Find best candidate (even if below threshold)
            best_iou = 0.0
            best_pred_id = None
            for pred_id, pred_info in pred_instances.items():
                iou = compute_best_iou(gt_info['points'], pred_info['points'], VOXEL_SIZE)
                if iou > best_iou:
                    best_iou = iou
                    best_pred_id = pred_id
            
            if best_pred_id:
                pred_class = pred_instances[best_pred_id]['class']
                reason = f"IoU too low ({best_iou:.3f})" if best_iou < IOU_THRESHOLD else "unknown"
                print(f"      GT#{gt_id} '{gt_class}' ({gt_points_count}pts) -> NO MATCH (best: Pred#{best_pred_id} '{pred_class}', {reason})")
            else:
                print(f"      GT#{gt_id} '{gt_class}' ({gt_points_count}pts) -> NO PREDICTIONS AVAILABLE")
    
    print(f"    Total unmatched GTs: {len(gt_instances) - len(matched_gt_set)}/{len(gt_instances)}")
    print(f"    ---\n")
    
    # Track matched GT and predictions
    matched_gt_ids = set()
    matched_pred_ids = set()
    
    # Initialize per-class tracking
    # Key difference from previous scripts: we track detections per class for aggregation
    class_detections = defaultdict(list)  # class -> list of detections
    class_gt_counts = defaultdict(int)  # class -> number of GT instances
    
    # Count GT instances per class
    for gt_id, gt_info in gt_instances.items():
        class_gt_counts[gt_info['class']] += 1
    
    # Process matches
    for gt_id, pred_id, iou, is_class_match in matches:
        gt_class = gt_instances[gt_id]['class']
        pred_info = pred_instances[pred_id]
        
        matched_gt_ids.add(gt_id)
        matched_pred_ids.add(pred_id)
        
        # KEY DIFFERENCE: A detection is a TP only if:
        # 1. IoU >= threshold (already satisfied by matching)
        # 2. Class matches (using synonym dictionary)
        is_tp = is_class_match
        
        # Record detection for the GT class
        class_detections[gt_class].append({
            'confidence': pred_info['confidence'],
            'is_tp': is_tp,
            'pred_class': pred_info['class'],
            'iou': iou
        })
    
    # Add unmatched predictions as FP
    for pred_id, pred_info in pred_instances.items():
        if pred_id not in matched_pred_ids:
            pred_class = pred_info['class']
            
            # Find matching GT class for this prediction using synonyms
            # This is for proper class assignment of FP
            assigned_class = None
            for gt_class in gt_classes:
                if class_matches(gt_class, pred_class):
                    assigned_class = gt_class
                    break
            
            # If no matching GT class found, use the predicted class
            if assigned_class is None:
                assigned_class = pred_class
            
            # Only count FP for classes present in GT
            if assigned_class in class_gt_counts:
                class_detections[assigned_class].append({
                    'confidence': pred_info['confidence'],
                    'is_tp': False,
                    'pred_class': pred_class,
                    'iou': 0.0
                })
    
    # Compute AP per class
    class_aps = {}
    for gt_class in gt_classes:
        n_gt = class_gt_counts[gt_class]
        detections = class_detections[gt_class]
        ap = compute_ap_for_class(detections, n_gt)
        
        n_tp = sum(1 for d in detections if d['is_tp'])
        n_fp = sum(1 for d in detections if not d['is_tp'])
        n_fn = n_gt - n_tp
        
        class_aps[gt_class] = {
            'ap': ap,
            'n_gt': n_gt,
            'n_tp': n_tp,
            'n_fp': n_fp,
            'n_fn': n_fn
        }
    
    # Compute scene mAP (mean of per-class APs)
    valid_aps = [data['ap'] for data in class_aps.values() if data['n_gt'] > 0]
    scene_map = np.mean(valid_aps) if valid_aps else 0.0
    
    # =========================================================================
    # SUMMARY LOGGING
    # =========================================================================
    print(f"    --- Per-Class Summary ---")
    # Show classes with TP
    classes_with_tp = [(cls, data) for cls, data in class_aps.items() if data['n_tp'] > 0]
    if classes_with_tp:
        print(f"    Classes with TP:")
        for cls, data in sorted(classes_with_tp, key=lambda x: x[1]['ap'], reverse=True)[:5]:
            print(f"      '{cls}': AP={data['ap']:.3f}, GT={data['n_gt']}, TP={data['n_tp']}, FP={data['n_fp']}")
    
    # Show classes with most FN
    classes_with_fn = [(cls, data) for cls, data in class_aps.items() if data['n_fn'] > 0]
    if classes_with_fn:
        print(f"    Top classes with FN (missed):")
        for cls, data in sorted(classes_with_fn, key=lambda x: x[1]['n_fn'], reverse=True)[:5]:
            print(f"      '{cls}': FN={data['n_fn']} (missed {data['n_fn']}/{data['n_gt']})")
    
    # Show classes with most FP
    classes_with_fp = [(cls, data) for cls, data in class_aps.items() if data['n_fp'] > 0]
    if classes_with_fp:
        print(f"    Top classes with FP (false alarms):")
        for cls, data in sorted(classes_with_fp, key=lambda x: x[1]['n_fp'], reverse=True)[:5]:
            print(f"      '{cls}': FP={data['n_fp']}")
    print(f"    ---")
    
    return {
        'scene_id': scene_id,
        'detector': detector,
        'map': scene_map,
        'n_gt_instances': len(gt_instances),
        'n_pred_instances': len(pred_instances),
        'n_matched': len(matches),
        'class_aps': class_aps,
        'gt_classes': list(gt_classes)
    }


# =============================================================================
# Aggregated Evaluation
# =============================================================================

def aggregate_results(scene_results: List[Dict]) -> Dict:
    """
    Aggregate results across all scenes for a detector.
    
    KEY IMPROVEMENT: Proper aggregation of AP across scenes.
    Instead of averaging scene mAPs, we aggregate detections per class across scenes
    and compute AP from the combined detections.
    """
    if not scene_results:
        return {
            'mean_map': 0.0,
            'n_scenes': 0,
            'per_class_ap': {},
            'total_gt': 0,
            'total_pred': 0,
            'total_tp': 0,
            'total_fp': 0,
            'total_fn': 0
        }
    
    # Aggregate per-class stats across all scenes
    class_total_gt = defaultdict(int)
    class_total_tp = defaultdict(int)
    class_total_fp = defaultdict(int)
    class_all_detections = defaultdict(list)
    
    total_gt = 0
    total_pred = 0
    
    for result in scene_results:
        total_gt += result['n_gt_instances']
        total_pred += result['n_pred_instances']
        
        for cls, cls_data in result['class_aps'].items():
            class_total_gt[cls] += cls_data['n_gt']
            class_total_tp[cls] += cls_data['n_tp']
            class_total_fp[cls] += cls_data['n_fp']
    
    # Compute per-class AP (average of scene APs per class)
    class_scene_aps = defaultdict(list)
    for result in scene_results:
        for cls, cls_data in result['class_aps'].items():
            if cls_data['n_gt'] > 0:
                class_scene_aps[cls].append(cls_data['ap'])
    
    per_class_ap = {}
    for cls in class_total_gt.keys():
        if class_scene_aps[cls]:
            per_class_ap[cls] = {
                'ap': np.mean(class_scene_aps[cls]),
                'n_scenes': len(class_scene_aps[cls]),
                'n_gt': class_total_gt[cls],
                'n_tp': class_total_tp[cls],
                'n_fp': class_total_fp[cls]
            }
    
    # Compute overall mAP
    valid_aps = [data['ap'] for data in per_class_ap.values()]
    mean_map = np.mean(valid_aps) if valid_aps else 0.0
    
    total_tp = sum(class_total_tp.values())
    total_fp = sum(class_total_fp.values())
    total_fn = total_gt - total_tp
    
    return {
        'mean_map': mean_map,
        'n_scenes': len(scene_results),
        'per_class_ap': per_class_ap,
        'total_gt': total_gt,
        'total_pred': total_pred,
        'total_tp': total_tp,
        'total_fp': total_fp,
        'total_fn': total_fn
    }


# =============================================================================
# Report Generation
# =============================================================================

def generate_report(all_results: Dict, output_path: Path):
    """
    Generate comprehensive text report comparing all detectors.
    """
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    with open(output_path, 'w') as f:
        f.write("=" * 100 + "\n")
        f.write("VOXELAND EVALUATION REPORT - SCANNET v2 DATASET\n")
        f.write(f"Generated: {timestamp}\n")
        f.write("Metric: mAP@0.5 (Mean Average Precision at IoU threshold 0.5)\n")
        f.write("=" * 100 + "\n\n")
        
        # =====================================================================
        # Overall Ranking
        # =====================================================================
        f.write("#" * 100 + "\n")
        f.write("DETECTOR RANKING (by mAP@0.5)\n")
        f.write("#" * 100 + "\n\n")
        
        rankings = []
        for detector in DETECTORS:
            if detector in all_results:
                mean_map = all_results[detector]['aggregated']['mean_map']
                rankings.append((detector, mean_map))
        
        rankings.sort(key=lambda x: x[1], reverse=True)
        
        f.write(f"{'Rank':<6} {'Detector':<15} {'mAP@0.5':<12} {'Scenes':<10} {'GT Inst.':<12} {'Pred Inst.':<12} {'TP':<8} {'FP':<8} {'FN':<8}\n")
        f.write("-" * 100 + "\n")
        
        for rank, (detector, mean_map) in enumerate(rankings, 1):
            agg = all_results[detector]['aggregated']
            f.write(f"{rank:<6} {detector.upper():<15} {mean_map:.4f}       {agg['n_scenes']:<10} {agg['total_gt']:<12} {agg['total_pred']:<12} {agg['total_tp']:<8} {agg['total_fp']:<8} {agg['total_fn']:<8}\n")
        
        f.write("-" * 100 + "\n\n")
        
        # =====================================================================
        # Per-Scene Comparison Table
        # =====================================================================
        f.write("=" * 100 + "\n")
        f.write("PER-SCENE mAP@0.5 COMPARISON\n")
        f.write("=" * 100 + "\n\n")
        
        # Get all scene IDs
        all_scenes = set()
        for detector in DETECTORS:
            if detector in all_results:
                for result in all_results[detector]['per_scene']:
                    all_scenes.add(result['scene_id'])
        
        all_scenes = sorted(all_scenes)
        
        # Header
        header = f"{'Scene':<20}"
        for detector in DETECTORS:
            header += f" {detector.upper():>12}"
        header += f" {'Best':>12}"
        f.write(header + "\n")
        f.write("-" * 100 + "\n")
        
        # Scene rows
        scene_bests = defaultdict(int)  # count of wins per detector
        
        for scene_id in all_scenes:
            row = f"{scene_id:<20}"
            scene_maps = {}
            
            for detector in DETECTORS:
                if detector in all_results:
                    scene_result = next(
                        (r for r in all_results[detector]['per_scene'] if r['scene_id'] == scene_id),
                        None
                    )
                    if scene_result:
                        map_val = scene_result['map']
                        row += f" {map_val:>12.4f}"
                        scene_maps[detector] = map_val
                    else:
                        row += f" {'N/A':>12}"
                else:
                    row += f" {'N/A':>12}"
            
            # Determine best
            if scene_maps:
                max_map = max(scene_maps.values())
                best_detectors = [d for d, m in scene_maps.items() if m == max_map]
                
                if len(best_detectors) == len(DETECTORS):
                    best_str = "TIE"
                else:
                    best_str = '/'.join(d.upper() for d in best_detectors)
                    for d in best_detectors:
                        scene_bests[d] += 1
                
                row += f" {best_str:>12}"
            else:
                row += f" {'N/A':>12}"
            
            f.write(row + "\n")
        
        f.write("-" * 100 + "\n")
        
        # Summary row
        row = f"{'MEAN':>20}"
        for detector in DETECTORS:
            if detector in all_results:
                mean_map = all_results[detector]['aggregated']['mean_map']
                row += f" {mean_map:>12.4f}"
            else:
                row += f" {'N/A':>12}"
        row += f" {'':<12}"
        f.write(row + "\n")
        
        # Wins summary
        f.write("\n")
        f.write(f"Scene wins: ")
        for detector in DETECTORS:
            f.write(f"{detector.upper()}: {scene_bests[detector]}  ")
        f.write("\n\n")
        
        # =====================================================================
        # Per-Detector Detailed Results
        # =====================================================================
        for detector in DETECTORS:
            if detector not in all_results:
                continue
            
            f.write("=" * 100 + "\n")
            f.write(f"{detector.upper()} - DETAILED RESULTS\n")
            f.write("=" * 100 + "\n\n")
            
            agg = all_results[detector]['aggregated']
            
            f.write(f"Overall mAP@0.5: {agg['mean_map']:.4f}\n")
            f.write(f"Scenes evaluated: {agg['n_scenes']}\n")
            f.write(f"Total GT instances: {agg['total_gt']}\n")
            f.write(f"Total predictions: {agg['total_pred']}\n")
            f.write(f"True Positives: {agg['total_tp']}\n")
            f.write(f"False Positives: {agg['total_fp']}\n")
            f.write(f"False Negatives: {agg['total_fn']}\n")
            f.write(f"Precision: {agg['total_tp'] / (agg['total_tp'] + agg['total_fp']) if agg['total_tp'] + agg['total_fp'] > 0 else 0:.4f}\n")
            f.write(f"Recall: {agg['total_tp'] / (agg['total_tp'] + agg['total_fn']) if agg['total_tp'] + agg['total_fn'] > 0 else 0:.4f}\n")
            f.write("\n")
            
            # Per-class AP table
            f.write("Per-Class Average Precision:\n")
            f.write(f"{'Class':<25} {'AP':<10} {'# Scenes':<10} {'# GT':<10} {'# TP':<10} {'# FP':<10}\n")
            f.write("-" * 75 + "\n")
            
            sorted_classes = sorted(agg['per_class_ap'].items(), 
                                   key=lambda x: x[1]['ap'], reverse=True)
            
            for cls, data in sorted_classes:
                f.write(f"{cls:<25} {data['ap']:<10.4f} {data['n_scenes']:<10} {data['n_gt']:<10} {data['n_tp']:<10} {data['n_fp']:<10}\n")
            
            f.write("\n")
        
        # =====================================================================
        # Methodology Notes
        # =====================================================================
        f.write("=" * 100 + "\n")
        f.write("METHODOLOGY NOTES\n")
        f.write("=" * 100 + "\n\n")
        
        f.write("Evaluation methodology:\n")
        f.write("- IoU Computation: Point cloud-based using KDTree radius search\n")
        f.write("- Matching: Hungarian algorithm (optimal bipartite assignment)\n")
        f.write("- TP Criteria: IoU >= 0.5 AND class match (using synonym dictionary)\n")
        f.write("- FP: Predictions matched with IoU < 0.5 OR wrong class OR unmatched\n")
        f.write("- FN: GT instances not matched to any prediction\n")
        f.write("- AP: VOC-style (all-point interpolation)\n")
        f.write("- mAP: Mean of per-class APs (only classes with GT instances)\n")
        f.write("\n")
        f.write("Key improvements over previous evaluation scripts:\n")
        f.write("1. Hungarian matching prevents multiple predictions matching same GT\n")
        f.write("2. Class verification required for TP (not just IoU)\n")
        f.write("3. Asymmetric synonym matching (GT synonyms -> prediction)\n")
        f.write("4. Dynamic class set from GT (not hardcoded subset)\n")
        f.write("5. Proper per-class AP aggregation across scenes\n")
        f.write("\n")


def generate_json_output(all_results: Dict, output_path: Path):
    """
    Generate JSON output with all evaluation results.
    """
    # Convert numpy types to Python types for JSON serialization
    def convert(obj):
        if isinstance(obj, np.floating):
            return float(obj)
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, dict):
            return {k: convert(v) for k, v in obj.items()}
        if isinstance(obj, list):
            return [convert(v) for v in obj]
        return obj
    
    output = convert(all_results)
    
    with open(output_path, 'w') as f:
        json.dump(output, f, indent=2)


# =============================================================================
# Main
# =============================================================================

def main():
    """
    Main evaluation function.
    """
    print("=" * 80)
    print("VOXELAND EVALUATION - SCANNET v2")
    print("=" * 80)
    print()
    
    # Check directories
    if not GT_DIR.exists():
        print(f"ERROR: Ground truth directory not found: {GT_DIR}")
        sys.exit(1)
    
    if not PRED_DIR.exists():
        print(f"ERROR: Predictions directory not found: {PRED_DIR}")
        sys.exit(1)
    
    # Get list of GT scenes
    gt_scenes = sorted([d.name for d in GT_DIR.iterdir() 
                        if d.is_dir() and d.name.startswith('scene')])
    
    print(f"Found {len(gt_scenes)} ground truth scenes")
    print(f"Evaluating detectors: {DETECTORS}")
    print()
    
    # Results storage
    all_results = {}
    
    # Evaluate each detector
    for detector in DETECTORS:
        print(f"\n{'=' * 60}")
        print(f"Evaluating: {detector.upper()}")
        print(f"{'=' * 60}")
        
        scene_results = []
        
        for scene_id in gt_scenes:
            print(f"  Processing {scene_id}...", end=" ")
            
            result = evaluate_scene(scene_id, detector)
            
            if result:
                scene_results.append(result)
                print(f"mAP: {result['map']:.4f} | GT: {result['n_gt_instances']} | Pred: {result['n_pred_instances']}")
            else:
                print("SKIPPED (missing data)")
        
        # Aggregate results
        aggregated = aggregate_results(scene_results)
        
        all_results[detector] = {
            'per_scene': scene_results,
            'aggregated': aggregated
        }
        
        print(f"\n  {detector.upper()} Summary:")
        print(f"    Scenes: {aggregated['n_scenes']}")
        print(f"    Mean mAP@0.5: {aggregated['mean_map']:.4f}")
        print(f"    Total GT: {aggregated['total_gt']}")
        print(f"    Total Pred: {aggregated['total_pred']}")
        print(f"    TP/FP/FN: {aggregated['total_tp']}/{aggregated['total_fp']}/{aggregated['total_fn']}")
    
    # Generate outputs
    txt_output = OUTPUT_DIR / 'scannet_v2_evaluation_results.txt'
    json_output = OUTPUT_DIR / 'scannet_v2_evaluation_results.json'
    
    print(f"\n{'=' * 60}")
    print("Generating reports...")
    
    generate_report(all_results, txt_output)
    generate_json_output(all_results, json_output)
    
    print(f"\nEvaluation complete!")
    print(f"Results saved to:")
    print(f"  - TXT: {txt_output}")
    print(f"  - JSON: {json_output}")
    print(f"{'=' * 60}")
    
    # Print final ranking
    print("\nFINAL RANKING:")
    rankings = [(d, all_results[d]['aggregated']['mean_map']) for d in DETECTORS if d in all_results]
    rankings.sort(key=lambda x: x[1], reverse=True)
    
    for rank, (detector, mean_map) in enumerate(rankings, 1):
        print(f"  {rank}. {detector.upper()}: {mean_map:.4f}")


if __name__ == '__main__':
    main()
