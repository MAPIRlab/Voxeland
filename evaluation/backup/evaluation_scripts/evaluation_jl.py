#!/usr/bin/env python3
"""
Evaluation Script for Voxeland using mAP@0.5 metric
Compares Voxeland detector outputs (detectron, talos, yoloe) against ground truth.
Supports both ScanNet and SceneNN datasets.
Computes mAP (mean Average Precision) at IoU threshold of 0.5.

This script follows the evaluation methodology from eval.py:
- Uses point cloud IoU calculation (not AABB)
- Applies axis transformations for proper alignment
- Computes AP per class and aggregates into mAP
- Includes uncertainty analysis using Shannon entropy

Usage:
    python3 evaluation_jl.py scannet
    python3 evaluation_jl.py scenenn
"""

import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Set
import numpy as np
from collections import defaultdict
from plyfile import PlyData
import open3d as o3d
from scipy.special import psi


# ============================================================================
# PLY Reading Functions (manual parsing for string support)
# ============================================================================

def read_voxeland_ply(ply_path: str) -> dict:
    """
    Read a Voxeland PLY file manually (supports string semantic_category field).
    
    Returns a dict with:
        - points: numpy array (N, 3) with x, y, z coordinates
        - colors: numpy array (N, 3) with r, g, b values
        - instance_ids: numpy array (N,) with instance IDs
        - semantic_categories: list of strings (N,) with semantic categories
    """
    points = []
    colors = []
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
                r, g, b = int(parts[3]), int(parts[4]), int(parts[5])
                instance_id = int(parts[6])
                # Category can have multiple words (e.g., "washing machine")
                semantic_category = ' '.join(parts[7:])
                
                points.append([x, y, z])
                colors.append([r, g, b])
                instance_ids.append(instance_id)
                semantic_categories.append(semantic_category)
    
    return {
        'points': np.array(points),
        'colors': np.array(colors),
        'instance_ids': np.array(instance_ids),
        'semantic_categories': semantic_categories
    }


def get_points_by_instance_id(ply_data: dict, instance_id: int) -> np.ndarray:
    """
    Extract points for a specific instance ID from manually parsed PLY data.
    
    Args:
        ply_data: Dictionary from read_voxeland_ply()
        instance_id: Instance ID to extract
    
    Returns:
        Numpy array (N, 3) with points
    """
    mask = ply_data['instance_ids'] == instance_id
    return ply_data['points'][mask]


def load_scenenn_gt_points(gt_ply_path: Path, instance_id: int, label: int) -> np.ndarray:
    """
    Load ground truth point cloud for a SceneNN instance.
    
    Args:
        gt_ply_path: Path to SceneNN GT PLY file (e.g., 011.ply)
        instance_id: Instance ID (not used, kept for consistency)
        label: Label ID to extract from PLY
    
    Returns:
        Numpy array (N, 3) with GT points
    """
    gt_ply = PlyData.read(str(gt_ply_path))
    gt_points = get_points_by_label(gt_ply, label, "label")
    
    # Voxelize GT at 0.02 resolution (as in eval.py line 75)
    gt_voxelized = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(gt_points))
    gt_voxelized = gt_voxelized.voxel_down_sample(voxel_size=0.02)
    gt_points = np.array(gt_voxelized.points)
    
    return gt_points


def load_scannet_gt_points(gt_ply_path: Path, aggregation_path: Path, 
                          segs_path: Path, instance_id: int) -> np.ndarray:
    """
    Load ground truth point cloud for a ScanNet instance.
    
    Args:
        gt_ply_path: Path to ScanNet GT PLY file (e.g., scene0000_01_vh_clean_2.ply)
        aggregation_path: Path to aggregation.json
        segs_path: Path to segs.json
        instance_id: Instance ID (objectId in aggregation.json)
    
    Returns:
        Numpy array (N, 3) with GT points
    """
    # Load aggregation to get segments for this instance
    with open(aggregation_path, 'r') as f:
        aggregation = json.load(f)
    
    # Find segments for this instance
    segments = None
    for seg_group in aggregation['segGroups']:
        if seg_group['objectId'] == instance_id:
            segments = set(seg_group['segments'])
            break
    
    if segments is None:
        return np.array([]).reshape(0, 3)
    
    # Load segmentation indices
    with open(segs_path, 'r') as f:
        segs = json.load(f)
    seg_indices = segs['segIndices']
    
    # Load PLY
    gt_ply = PlyData.read(str(gt_ply_path))
    x = np.array(gt_ply.elements[0].data["x"])
    y = np.array(gt_ply.elements[0].data["y"])
    z = np.array(gt_ply.elements[0].data["z"])
    
    # Find vertices belonging to this instance's segments
    vertex_mask = np.array([seg_id in segments for seg_id in seg_indices])
    
    # Extract points
    gt_points = np.stack([x[vertex_mask], y[vertex_mask], z[vertex_mask]], axis=1)
    
    # Voxelize GT at 0.02 resolution (as in eval.py line 75)
    if gt_points.shape[0] > 0:
        gt_voxelized = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(gt_points))
        gt_voxelized = gt_voxelized.voxel_down_sample(voxel_size=0.02)
        gt_points = np.array(gt_voxelized.points)
    
    return gt_points


# ============================================================================
# Utility Functions (from utils.py)
# ============================================================================

def get_points_by_label(ply, label, label_field):
    """Extract points from PLY for a specific label."""
    valid_points = ply.elements[0].data[label_field] == int(label)
    
    x = np.array(ply.elements[0].data["x"][valid_points]).reshape(-1)
    y = np.array(ply.elements[0].data["y"][valid_points]).reshape(-1)
    z = np.array(ply.elements[0].data["z"][valid_points]).reshape(-1)

    points = np.array([x, y, z]).T

    return points


def get_points_indexes_by_label(ply, label, label_field):
    """Get boolean mask for points with specific label."""
    valid_points = ply.elements[0].data[label_field] == int(label)
    return valid_points


def get_full_pointcloud(ply):
    """Convert PLY data to Open3D point cloud."""
    x = np.array(ply.elements[0].data["x"]).reshape(-1)
    y = np.array(ply.elements[0].data["y"]).reshape(-1)
    z = np.array(ply.elements[0].data["z"]).reshape(-1)

    points = np.array([x, y, z]).T
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    return pcd


def compute_iou_3d_aabb(bbox1: Dict, bbox2: Dict) -> float:
    """
    Compute 3D Intersection over Union (IoU) between two axis-aligned bounding boxes.
    
    Args:
        bbox1, bbox2: Dictionaries with keys x_min, x_max, y_min, y_max, z_min, z_max
    
    Returns:
        IoU value between 0 and 1
    """
    # Calculate intersection
    x_overlap = max(0, min(bbox1['x_max'], bbox2['x_max']) - max(bbox1['x_min'], bbox2['x_min']))
    y_overlap = max(0, min(bbox1['y_max'], bbox2['y_max']) - max(bbox1['y_min'], bbox2['y_min']))
    z_overlap = max(0, min(bbox1['z_max'], bbox2['z_max']) - max(bbox1['z_min'], bbox2['z_min']))
    
    intersection = x_overlap * y_overlap * z_overlap
    
    # Calculate volumes
    volume1 = (bbox1['x_max'] - bbox1['x_min']) * \
              (bbox1['y_max'] - bbox1['y_min']) * \
              (bbox1['z_max'] - bbox1['z_min'])
    
    volume2 = (bbox2['x_max'] - bbox2['x_min']) * \
              (bbox2['y_max'] - bbox2['y_min']) * \
              (bbox2['z_max'] - bbox2['z_min'])
    
    # Calculate union
    union = volume1 + volume2 - intersection
    
    if union <= 0:
        return 0.0
    
    return intersection / union


def compute_iou(pc1, pc2, voxel_size):
    """
    Compute IoU between two point clouds using KDTree radius search.
    This is the primary IoU method from eval.py.
    """
    # Convert numpy arrays to Open3D point clouds
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(pc1)

    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(pc2)

    if pc1.shape[0] == 0 or pc2.shape[0] == 0:
        return 0
    
    if pc1.shape[0] >= pc2.shape[0]:
        bigger_pc = pcd1
        smaller_pc = pcd2
    else:
        bigger_pc = pcd2
        smaller_pc = pcd1

    # Build KDTree for the bigger point cloud
    bigger_pc_tree = o3d.geometry.KDTreeFlann(bigger_pc)

    # Count the number of points in smaller_pc with a nearby point in bigger_pc
    count_close_points = 0

    for point in smaller_pc.points:
        [k, idx, _] = bigger_pc_tree.search_radius_vector_3d(point, 4*voxel_size)
        if k > 0:
            count_close_points += 1
        
    iou = float(count_close_points) / (pc1.shape[0] + pc2.shape[0] - count_close_points)
    return iou


def compute_iou_v2(pc1, pc2, voxel_size):
    """
    Alternative IoU computation with voxel downsampling.
    This is the secondary IoU method from eval.py.
    """
    # Convert numpy arrays to Open3D point clouds
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(pc1)

    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(pc2)

    if pc1.shape[0] == 0 or pc2.shape[0] == 0:
        return 0
    
    pcd1 = pcd1.voxel_down_sample(voxel_size=0.05)
    pcd2 = pcd2.voxel_down_sample(voxel_size=0.05)

    pc1 = np.array(pcd1.points)
    pc2 = np.array(pcd2.points)
    
    if pc1.shape[0] >= pc2.shape[0]:
        bigger_pc = pcd1
        smaller_pc = pcd2
    else:
        bigger_pc = pcd2
        smaller_pc = pcd1

    # Build KDTree
    bigger_pc_tree = o3d.geometry.KDTreeFlann(bigger_pc)

    count_close_points = 0

    for point in smaller_pc.points:
        [k, idx, _] = bigger_pc_tree.search_radius_vector_3d(point, np.sqrt(2)*0.05+0.0005)
        if k > 0:
            count_close_points += 1
        
    iou = float(count_close_points) / (pc1.shape[0] + pc2.shape[0] - count_close_points)
    return iou


def expected_shannon_entropy(alpha):
    """
    Compute the expected Shannon entropy of distributions drawn from a Dirichlet distribution.
    Used for uncertainty analysis.
    
    Parameters:
    alpha (array-like): Parameters of the Dirichlet distribution
    
    Returns:
    float: Expected Shannon entropy
    """
    alpha = np.array(alpha)
    A = np.sum(alpha)
    expected_entropy = psi(A) - (1/A) * np.sum(alpha * psi(alpha))
    
    return expected_entropy


def voc_ap(rec, prec, use_07_metric=False):
    """
    Compute VOC AP given precision and recall.
    If use_07_metric is true, uses the VOC 07 11 point method (default:False).
    """
    if use_07_metric:
        # 11 point metric
        ap = 0.
        for t in np.arange(0., 1.1, 0.1):
            if np.sum(rec >= t) == 0:
                p = 0
            else:
                p = np.max(prec[rec >= t])
            ap = ap + p / 11.
    else:
        # correct AP calculation
        # first append sentinel values at the end
        mrec = np.concatenate(([0.], rec, [1.]))
        mpre = np.concatenate(([0.], prec, [0.]))

        # compute the precision envelope
        for i in range(mpre.size - 1, 0, -1):
            mpre[i - 1] = np.maximum(mpre[i - 1], mpre[i])

        # to calculate area under PR curve, look for points
        # where X axis (recall) changes value
        i = np.where(mrec[1:] != mrec[:-1])[0]

        # and sum (\Delta recall) * prec
        ap = np.sum((mrec[i + 1] - mrec[i]) * mpre[i + 1])
    return ap


def rotation_matrix(angle, axis):
    """
    Create a rotation matrix for rotation around an axis.
    Used for coordinate system transformations.
    """
    axis = np.asarray(axis)
    axis = axis / np.linalg.norm(axis)
    a = np.cos(angle / 2.0)
    b, c, d = -axis * np.sin(angle / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac), 0],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab), 0],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc, 0],
                     [0, 0, 0, 1]])


def extract_scores_from_json(instance_data: Dict) -> Tuple[List[str], List[float]]:
    """
    Extract and normalize category scores from Voxeland JSON instance data.
    
    Args:
        instance_data: Dictionary with either "scores" or "results" field
    
    Returns:
        Tuple of (categories, normalized_scores)
    """
    categories = []
    scores = []
    
    # Check for "scores" field (new format) or "results" field (old format)
    score_dict = instance_data.get("scores") or instance_data.get("results") or {}
    
    # Filter out "unknown" category
    for category, score in score_dict.items():
        if category.lower() != "unknown":
            categories.append(category)
            scores.append(score)
    
    if len(categories) == 0:
        return [], []
    
    # Sort by score descending
    combined = list(zip(categories, scores))
    sorted_combined = sorted(combined, key=lambda x: x[1], reverse=True)
    
    categories, scores = zip(*sorted_combined)
    
    # Normalize scores to get probabilities
    total = sum(scores)
    if total > 0:
        scores = [s / total for s in scores]
    else:
        scores = [0.0] * len(scores)
    
    return list(categories), list(scores)


# ============================================================================
# Class Mapping Dictionaries
# ============================================================================

# COCO classes to NYU40 classes (for ScanNet)
COCO_TO_NYU40 = {
    "bed": "bed",
    "chair": "chair",
    "couch": "sofa",
    "sofa": "sofa",
    "dining table": "table",
    "table": "table",
    "book": "books",
    "books": "books",
    "refrigerator": "refrigerator",
    "tv": "television",
    "television": "television",
    "toilet": "toilet",
    "handbag": "bag",
    "bag": "bag"
}

# SceneNN class mappings
SCENENN_CLASS_MAP = {
    "bed": "bed",
    "chair": "chair",
    "sofa": "sofa",
    "table": "table",
    "desk": "table",
    "books": "books",
    "book": "books",
    "refridgerator": "refrigerator",
    "refrigerator": "refrigerator",
    "fridge": "refrigerator",
    "TV": "television",
    "television": "television",
    "toilet": "toilet",
    "bag": "bag"
}


def normalize_class_name(class_name: str, dataset: str) -> str:
    """
    Normalize class names according to dataset conventions.
    
    Args:
        class_name: Original class name
        dataset: 'scannet' or 'scenenn'
    
    Returns:
        Normalized class name
    """
    class_lower = class_name.lower().strip()
    
    if dataset == 'scannet':
        return COCO_TO_NYU40.get(class_lower, class_lower)
    else:  # scenenn
        return SCENENN_CLASS_MAP.get(class_lower, class_lower)


def check_class_match(gt_class: str, gt_synonyms: List[str], pred_class: str, dataset: str) -> bool:
    """
    Check if predicted class matches ground truth class (considering synonyms and normalization).
    
    Args:
        gt_class: Ground truth class name
        gt_synonyms: List of synonym names
        pred_class: Predicted class name
        dataset: 'scannet' or 'scenenn'
    
    Returns:
        True if match, False otherwise
    """
    # Normalize prediction
    pred_normalized = normalize_class_name(pred_class, dataset)
    gt_normalized = normalize_class_name(gt_class, dataset)
    
    # Direct match
    if pred_normalized == gt_normalized:
        return True
    
    # Check synonyms
    for synonym in gt_synonyms:
        synonym_normalized = normalize_class_name(synonym, dataset)
        if pred_normalized == synonym_normalized:
            return True
    
    return False


# ============================================================================
# Main Evaluation Functions
# ============================================================================

def evaluate_scene(scene_id: str, gt_data: Dict, pred_ply_path: Path, 
                   pred_json_path: Path, detector: str, dataset: str,
                   gt_ply_path: Path = None, gt_xml_path: Path = None,
                   gt_aggregation_path: Path = None, gt_segs_path: Path = None,
                   voxel_size: float = 0.02, iou_threshold: float = 0.5) -> Dict:
    """
    Evaluate a single scene using mAP@0.5 methodology from eval.py.
    
    Args:
        scene_id: Scene identifier
        gt_data: Ground truth data with instances
        pred_ply_path: Path to prediction PLY file
        pred_json_path: Path to prediction JSON file
        detector: Detector name
        dataset: 'scannet' or 'scenenn'
        gt_ply_path: Path to GT PLY file (for point cloud extraction)
        gt_xml_path: Path to GT XML file (for SceneNN label mapping)
        gt_aggregation_path: Path to aggregation.json (for ScanNet)
        gt_segs_path: Path to segs.json (for ScanNet)
        voxel_size: Voxel size for IoU computation
        iou_threshold: IoU threshold for TP/FP classification
    
    Returns:
        Dictionary with evaluation results
    """
    # Load prediction data - use manual PLY reading to support string fields
    pred_ply_data = read_voxeland_ply(str(pred_ply_path))
    with open(pred_json_path, 'r') as f:
        pred_json = json.load(f)
    
    # Create a dictionary mapping instance_id -> instance data for easy lookup
    pred_json_by_id = {}
    for instance in pred_json['instances']:
        pred_json_by_id[instance['instance_id']] = instance
    
    # Get full point cloud from manually parsed data
    pred_pcd = o3d.geometry.PointCloud()
    pred_pcd.points = o3d.utility.Vector3dVector(pred_ply_data['points'])
    
    # Apply axis transformation (from eval.py line 65)
    # Align own-axis to GT-axis: 90 degree rotation around X axis
    Rx = rotation_matrix(np.radians(90), (1, 0, 0))
    pred_points = np.linalg.inv(Rx)[:3,:3] @ np.array(pred_pcd.points).T
    pred_pcd.points = o3d.utility.Vector3dVector(pred_points.T)
    
    # Build GT instances dictionary with point clouds
    gt_instances = {}
    
    if dataset == 'scenenn':
        # For SceneNN, we need to load from XML to get proper label IDs
        if gt_xml_path and gt_xml_path.exists() and gt_ply_path and gt_ply_path.exists():
            import xml.etree.ElementTree as ET
            tree = ET.parse(gt_xml_path)
            root = tree.getroot()
            
            valid_classes = ["bed", "chair", "sofa", "table", "desk", "books", "book", 
                           "refridgerator", "refrigerator", "fridge", "television", "TV", 
                           "toilet", "bag"]
            
            for child in root:
                nyu_class = child.attrib.get("nyu_class")
                label_id = child.attrib.get("id")
                
                if not nyu_class or not label_id:
                    continue
                
                # Check if it's a valid class
                if nyu_class not in valid_classes:
                    continue
                
                # Load GT point cloud for this label
                gt_points = load_scenenn_gt_points(gt_ply_path, 0, int(label_id))
                
                if gt_points.shape[0] == 0:
                    continue
                
                # Create instance entry (using XML id as gt_id)
                gt_instances[label_id] = {
                    'class': nyu_class,
                    'synonyms': [],  # We don't have synonyms from XML
                    'points': gt_points
                }
    else:  # scannet
        # For ScanNet, use the JSON data with instance_id mapping
        for gt_inst in gt_data['instances']:
            gt_id = gt_inst['instance_id']
            gt_class = gt_inst['class_name']
            gt_synonyms = gt_inst.get('synonyms', [])
            
            # Load GT point cloud
            gt_points = np.array([]).reshape(0, 3)
            if gt_ply_path and gt_ply_path.exists() and gt_aggregation_path and gt_segs_path:
                gt_points = load_scannet_gt_points(gt_ply_path, gt_aggregation_path, 
                                                   gt_segs_path, gt_id)
            
            gt_instances[gt_id] = {
                'class': gt_class,
                'synonyms': gt_synonyms,
                'points': gt_points
            }
    
    # Build prediction instances dictionary with point clouds and scores
    pred_instances = {}
    for pred_inst in pred_json_by_id.values():
        pred_id = pred_inst['instance_id']
        
        # Skip instance_id 0 (unknown/background)
        if pred_id == 0:
            continue
        
        pred_class = pred_inst['class_name']
        
        # Skip unknown class
        if pred_class.lower() == 'unknown':
            continue
        
        # Extract scores
        categories, scores = extract_scores_from_json(pred_inst)
        
        if len(categories) == 0:
            continue
        
        # Get AABB for this instance
        pred_aabb = pred_inst.get('aabb')
        if not pred_aabb:
            continue
        
        # Extract point cloud for this instance from prediction PLY
        # Get indexes of points with this instance_id
        pred_indexes = (pred_ply_data['instance_ids'] == pred_id)
        pred_points_raw = pred_ply_data['points'][pred_indexes]
        
        # Apply the same axis transformation as the full point cloud
        if pred_points_raw.shape[0] > 0:
            pred_points_transformed = np.linalg.inv(Rx)[:3,:3] @ pred_points_raw.T
            pred_points = pred_points_transformed.T
        else:
            pred_points = pred_points_raw
        
        pred_instances[pred_id] = {
            'class': pred_class,
            'categories': categories,
            'scores': scores,
            'confidence': scores[0] if len(scores) > 0 else 0.0,
            'aabb': pred_aabb,
            'points': pred_points,
            'score_dict': dict(zip(categories, scores))
        }
    
    # Match GT to predictions using point cloud IoU (same as eval.py)
    pairs = {}
    
    for gt_id, gt_info in gt_instances.items():
        gt_points = gt_info['points']
        
        pairs[gt_id] = {
            'gt_class': gt_info['class'],
            'gt_synonyms': gt_info['synonyms'],
            'gt_n_points': gt_points.shape[0],
            'correspondences': {}
        }
        
        # Skip if no GT points (shouldn't happen but safety check)
        if gt_points.shape[0] == 0:
            continue
        
        # Compute IoU with all predictions using point clouds
        for pred_id, pred_info in pred_instances.items():
            pred_points = pred_info['points']
            
            # Skip if no prediction points
            if pred_points.shape[0] == 0:
                continue
            
            # Compute point cloud IoU: max of two methods (as in eval.py line 95)
            iou = max(compute_iou(pred_points, gt_points, voxel_size),
                     compute_iou_v2(pred_points, gt_points, voxel_size))
            
            pairs[gt_id]['correspondences'][pred_id] = iou
    
    # Prepare data structures for mAP calculation
    # Get list of all valid classes from GT
    valid_classes = set()
    for gt_info in gt_instances.values():
        gt_class_normalized = normalize_class_name(gt_info['class'], dataset)
        valid_classes.add(gt_class_normalized)
    
    average_precisions = {c: {"y_gt": [], "y_pred": [], "confidence": [], "uncertainty": []} 
                         for c in valid_classes}
    
    predictions_summary = {}
    gt_summary = {}
    
    # Mark predictions that will be matched
    for pred_id, pred_info in pred_instances.items():
        predictions_summary[pred_id] = {
            'category': pred_info['categories'][0],
            'confidence': pred_info['confidence'],
            'correct': False
        }
    
    # Match each GT to best prediction (greedy by IoU)
    for gt_id, pair_info in pairs.items():
        if len(pair_info['correspondences']) == 0:
            # False negative - no prediction matched
            gt_class_normalized = normalize_class_name(pair_info['gt_class'], dataset)
            gt_summary[gt_id] = {'category': gt_class_normalized, 'correct': False}
            continue
        
        # Get best correspondence
        best_pred_id = max(pair_info['correspondences'], 
                          key=pair_info['correspondences'].get)
        best_iou = pair_info['correspondences'][best_pred_id]
        
        gt_class_normalized = normalize_class_name(pair_info['gt_class'], dataset)
        pred_info = pred_instances[best_pred_id]
        
        # For mAP calculation, we evaluate over GT classes
        best_class = gt_class_normalized
        
        # Get confidence and uncertainty
        confidence = pred_info['confidence']
        
        # Compute uncertainty (Shannon entropy from Dirichlet)
        # Use raw scores before normalization as alpha parameters
        alpha_vector = []
        score_dict_raw = pred_json_by_id[best_pred_id].get('scores') or \
                        pred_json_by_id[best_pred_id].get('results') or {}
        for score in score_dict_raw.values():
            if isinstance(score, (int, float)):
                alpha_vector.append(score)
        
        uncertainty = expected_shannon_entropy(alpha_vector) if len(alpha_vector) > 0 else 0.0
        
        # Classify as TP or FP based on IoU threshold
        if best_iou >= iou_threshold:
            # True Positive
            average_precisions[best_class]["y_gt"].append(1)
            average_precisions[best_class]["y_pred"].append(1)
            average_precisions[best_class]["confidence"].append(confidence)
            average_precisions[best_class]["uncertainty"].append(uncertainty)
            
            gt_summary[gt_id] = {'category': gt_class_normalized, 'correct': True}
            predictions_summary[best_pred_id]['correct'] = True
        else:
            # False Positive (IoU too low)
            average_precisions[best_class]["y_gt"].append(1)
            average_precisions[best_class]["y_pred"].append(0)
            average_precisions[best_class]["confidence"].append(confidence)
            average_precisions[best_class]["uncertainty"].append(uncertainty)
            
            gt_summary[gt_id] = {'category': gt_class_normalized, 'correct': False}
    
    # Add False Positives (predictions not matched to any GT)
    for pred_id, pred_summary in predictions_summary.items():
        if not pred_summary['correct']:
            pred_category = normalize_class_name(pred_summary['category'], dataset)
            
            # Only add FP if the class is in our valid classes
            if pred_category in average_precisions:
                average_precisions[pred_category]["y_gt"].append(0)
                average_precisions[pred_category]["y_pred"].append(1)
                average_precisions[pred_category]["confidence"].append(pred_summary['confidence'])
                
                # Get uncertainty for this prediction
                pred_info = pred_instances.get(pred_id)
                if pred_info:
                    score_dict_raw = pred_json_by_id[pred_id].get('scores') or \
                                    pred_json_by_id[pred_id].get('results') or {}
                    alpha_vector = [s for s in score_dict_raw.values() if isinstance(s, (int, float))]
                    uncertainty = expected_shannon_entropy(alpha_vector) if len(alpha_vector) > 0 else 0.0
                    average_precisions[pred_category]["uncertainty"].append(uncertainty)
    
    # Compute AP for each class
    class_aps = {}
    mean_ap = 0.0
    total_classes = 0
    
    for obj_class in average_precisions.keys():
        # Count GT objects of this class
        count_gt = sum(1 for obj in gt_summary.values() if obj["category"] == obj_class)
        
        if count_gt == 0:
            continue
        
        total_classes += 1
        
        # Sort by confidence descending
        sorted_ind = np.argsort(-np.array(average_precisions[obj_class]["confidence"]))
        
        y_true = np.array(average_precisions[obj_class]["y_gt"])[sorted_ind]
        y_scores = np.array(average_precisions[obj_class]["y_pred"])[sorted_ind]
        
        # Compute precision and recall
        tp = (y_true == 1) & (y_scores == 1)
        fp = (y_true == 0) & (y_scores == 1)
        
        tp = np.cumsum(tp)
        fp = np.cumsum(fp)
        npos = np.array(y_true).sum()
        rec = tp / float(npos)
        prec = tp / np.maximum(tp + fp, np.finfo(np.float64).eps)
        
        # Compute AP
        this_ap = voc_ap(rec, prec, use_07_metric=False)
        
        class_aps[obj_class] = {
            'ap': this_ap,
            'num_gt': int(npos),
            'num_pred': len(y_scores)
        }
        
        mean_ap += this_ap
    
    # Compute final mAP
    final_map = float(mean_ap) / total_classes if total_classes > 0 else 0.0
    
    return {
        'scene_id': scene_id,
        'num_gt_instances': len(gt_instances),
        'num_pred_instances': len(pred_instances),
        'map_0.5': final_map,
        'num_classes_evaluated': total_classes,
        'class_aps': class_aps
    }


def aggregate_map_results(scene_results: List[Dict]) -> Dict:
    """
    Aggregate mAP results across all scenes.
    
    Args:
        scene_results: List of per-scene evaluation results
    
    Returns:
        Aggregated metrics
    """
    if len(scene_results) == 0:
        return {
            'num_scenes': 0,
            'mean_map_0.5': 0.0,
            'total_gt_instances': 0,
            'total_pred_instances': 0,
            'class_aps': {}
        }
    
    # Aggregate per-class AP across scenes
    all_class_aps = defaultdict(list)
    
    for scene_result in scene_results:
        for class_name, class_data in scene_result['class_aps'].items():
            all_class_aps[class_name].append(class_data['ap'])
    
    # Compute mean AP per class
    class_mean_aps = {}
    for class_name, aps in all_class_aps.items():
        class_mean_aps[class_name] = {
            'mean_ap': float(np.mean(aps)),
            'num_scenes': len(aps)
        }
    
    # Compute overall mAP (mean of scene mAPs)
    mean_map = np.mean([r['map_0.5'] for r in scene_results])
    
    return {
        'num_scenes': len(scene_results),
        'mean_map_0.5': float(mean_map),
        'total_gt_instances': sum(r['num_gt_instances'] for r in scene_results),
        'total_pred_instances': sum(r['num_pred_instances'] for r in scene_results),
        'class_aps': class_mean_aps
    }


def write_text_report(all_results: Dict, scenes: List[str], output_file: Path, dataset_name: str):
    """
    Write a human-readable text report with mAP metrics and detector ranking.
    """
    with open(output_file, 'w') as f:
        f.write("="*80 + "\n")
        f.write(f"{dataset_name.upper()} EVALUATION RESULTS - VOXELAND (mAP@0.5)\n")
        f.write("="*80 + "\n\n")
        
        # Detector ranking by mAP@0.5
        f.write("#" * 80 + "\n")
        f.write("###" + " " * 23 + "DETECTOR RANKING (mAP@0.5)" + " " * 23 + "###\n")
        f.write("#" * 80 + "\n\n")
        
        rankings = []
        for detector in ['detectron', 'talos', 'yoloe']:
            if detector in all_results:
                overall = all_results[detector]['overall_metrics']
                map_score = overall['mean_map_0.5']
                rankings.append((detector, map_score))
        
        rankings.sort(key=lambda x: x[1], reverse=True)
        
        f.write(f"{'Rank':<6} {'Detector':<12} {'mAP@0.5':>12}\n")
        f.write("-"*80 + "\n")
        for rank, (detector, map_score) in enumerate(rankings, 1):
            f.write(f"{rank:<6} {detector.upper():<12} {map_score:>12.4f}\n")
        f.write("-"*80 + "\n\n")
        
        # Overall results for each detector
        for detector in ['detectron', 'talos', 'yoloe']:
            if detector not in all_results:
                continue
                
            overall = all_results[detector]['overall_metrics']
            
            f.write("="*80 + "\n")
            f.write(f"{detector.upper()} - OVERALL RESULTS\n")
            f.write("="*80 + "\n")
            f.write(f"Scenes Evaluated: {overall['num_scenes']}\n")
            f.write(f"Total Ground Truth Instances: {overall['total_gt_instances']}\n")
            f.write(f"Total Predicted Instances: {overall['total_pred_instances']}\n")
            f.write(f"Mean Average Precision (mAP@0.5): {overall['mean_map_0.5']:.4f}\n\n")
            
            # Per-class AP
            if overall['class_aps']:
                f.write("Per-Class Average Precision:\n")
                f.write(f"{'Class':<20} {'Mean AP':>12} {'Num Scenes':>12}\n")
                f.write("-"*80 + "\n")
                
                # Sort by mean AP descending
                sorted_classes = sorted(overall['class_aps'].items(), 
                                       key=lambda x: x[1]['mean_ap'], reverse=True)
                
                for class_name, class_data in sorted_classes:
                    f.write(f"{class_name:<20} {class_data['mean_ap']:>12.4f} {class_data['num_scenes']:>12}\n")
                f.write("\n")
        
        # Per-scene comparison
        f.write("="*80 + "\n")
        f.write("PER-SCENE mAP@0.5 COMPARISON\n")
        f.write("="*80 + "\n")
        f.write(f"{'Scene':<15} {'Detectron':>12} {'TALOS':>12} {'YOLOE':>12} {'Best':>12}\n")
        f.write("-"*80 + "\n")
        
        for scene in scenes:
            scene_scores = {}
            
            for detector in ['detectron', 'talos', 'yoloe']:
                if detector not in all_results:
                    scene_scores[detector] = None
                    continue
                scene_results = [s for s in all_results[detector]['per_scene_results'] if s['scene_id'] == scene]
                if scene_results:
                    map_score = scene_results[0]['map_0.5']
                    scene_scores[detector] = map_score
                else:
                    scene_scores[detector] = None
            
            # Find best detector(s)
            valid_scores = {k: v for k, v in scene_scores.items() if v is not None}
            if not valid_scores:
                best_str = 'N/A'
            else:
                max_score = max(valid_scores.values())
                best_detectors = [k for k, v in valid_scores.items() if v == max_score]
                
                if len(best_detectors) == 3:
                    best_str = '-'
                elif len(best_detectors) == 2:
                    best_str = '/'.join(best_detectors)
                else:
                    best_str = best_detectors[0]
            
            # Write row
            det_str = f"{scene_scores['detectron']:.3f}" if scene_scores['detectron'] is not None else "N/A"
            tal_str = f"{scene_scores['talos']:.3f}" if scene_scores['talos'] is not None else "N/A"
            yol_str = f"{scene_scores['yoloe']:.3f}" if scene_scores['yoloe'] is not None else "N/A"
            
            f.write(f"{scene:<15} {det_str:>12} {tal_str:>12} {yol_str:>12} {best_str:>12}\n")
        
        f.write("-"*80 + "\n")


def get_prediction_filename(scene_folder: str, detector: str) -> Tuple[str, str]:
    """
    Get the prediction PLY and JSON filenames based on scene folder name.
    
    Args:
        scene_folder: Scene folder name (e.g., "scannet_scene0000_01", "scenenn_011")
        detector: Detector name (detectron, talos, yoloe)
    
    Returns:
        Tuple of (ply_filename, json_filename)
    """
    base_name = f"voxeland_semantic_map_{detector}_{scene_folder}"
    return f"{base_name}.ply", f"{base_name}.json"


def main():
    """
    Main evaluation function.
    """
    # Check command line arguments
    if len(sys.argv) != 2 or sys.argv[1].lower() not in ['scannet', 'scenenn']:
        print("Usage: python3 evaluation_jl.py <dataset>")
        print("  dataset: 'scannet' or 'scenenn'")
        sys.exit(1)
    
    dataset = sys.argv[1].lower()
    
    # Define paths
    script_dir = Path(__file__).parent
    gt_dir = script_dir / f'{dataset}_groundtruth'
    pred_dir = script_dir / 'voxeland_output'
    output_json = script_dir / f'{dataset}_evaluation_map_results.json'
    output_txt = script_dir / f'{dataset}_evaluation_map_results.txt'
    
    # Check if directories exist
    if not gt_dir.exists():
        print(f"ERROR: Ground truth directory not found: {gt_dir}")
        sys.exit(1)
    
    if not pred_dir.exists():
        print(f"ERROR: Prediction directory not found: {pred_dir}")
        sys.exit(1)
    
    # Get list of scenes from ground truth directory
    gt_scenes = sorted([d.name for d in gt_dir.iterdir() if d.is_dir()])
    detectors = ['detectron', 'talos', 'yoloe']
    
    print(f"Starting {dataset.upper()} evaluation (mAP@0.5)...")
    print(f"Found {len(gt_scenes)} ground truth scenes: {gt_scenes}")
    print(f"Evaluating {len(detectors)} detectors: {detectors}")
    print()
    
    # Results structure
    all_results = {}
    
    # Evaluation parameters
    voxel_size = 0.02
    iou_threshold = 0.5
    
    # Evaluate each detector
    for detector in detectors:
        print(f"\n{'='*60}")
        print(f"Evaluating detector: {detector.upper()}")
        print(f"{'='*60}")
        
        detector_scene_results = []
        
        for gt_scene_id in gt_scenes:
            # Map GT scene name to voxeland output folder name
            if dataset == 'scannet':
                voxeland_scene_folder = f"scannet_{gt_scene_id}"
            else:  # scenenn
                voxeland_scene_folder = f"scenenn_{gt_scene_id}"
            
            # Load ground truth
            gt_file = gt_dir / gt_scene_id / f"{gt_scene_id}_gt_instances_aabb_synonyms.json"
            if not gt_file.exists():
                print(f"  [WARNING] Ground truth not found for {gt_scene_id}, skipping...")
                continue
            
            with open(gt_file, 'r') as f:
                gt_data = json.load(f)
            
            # Get prediction filenames
            pred_ply_filename, pred_json_filename = get_prediction_filename(voxeland_scene_folder, detector)
            pred_ply_path = pred_dir / voxeland_scene_folder / pred_ply_filename
            pred_json_path = pred_dir / voxeland_scene_folder / pred_json_filename
            
            if not pred_ply_path.exists():
                print(f"  [WARNING] Prediction PLY not found: {pred_ply_filename}, skipping...")
                continue
            
            if not pred_json_path.exists():
                print(f"  [WARNING] Prediction JSON not found: {pred_json_filename}, skipping...")
                continue
            
            # Get GT PLY paths for point cloud extraction
            gt_ply_path = None
            gt_xml_path = None
            gt_aggregation_path = None
            gt_segs_path = None
            
            if dataset == 'scannet':
                # ScanNet GT files
                gt_ply_path = gt_dir / gt_scene_id / f"{gt_scene_id}_vh_clean_2.ply"
                gt_aggregation_path = gt_dir / gt_scene_id / f"{gt_scene_id}.aggregation.json"
                gt_segs_path = gt_dir / gt_scene_id / f"{gt_scene_id}_vh_clean_2.0.010000.segs.json"
                
                if not gt_ply_path.exists():
                    print(f"  [WARNING] GT PLY not found: {gt_ply_path.name}, using AABB IoU fallback...")
                    gt_ply_path = None
            else:  # scenenn
                # SceneNN GT files
                gt_ply_path = gt_dir / gt_scene_id / f"{gt_scene_id}.ply"
                gt_xml_path = gt_dir / gt_scene_id / f"{gt_scene_id}.xml"
                
                if not gt_ply_path.exists():
                    print(f"  [WARNING] GT PLY not found: {gt_ply_path.name}, using AABB IoU fallback...")
                    gt_ply_path = None
            
            # Evaluate scene
            try:
                scene_result = evaluate_scene(
                    gt_scene_id, gt_data, pred_ply_path, pred_json_path, 
                    detector, dataset,
                    gt_ply_path=gt_ply_path,
                    gt_xml_path=gt_xml_path,
                    gt_aggregation_path=gt_aggregation_path,
                    gt_segs_path=gt_segs_path,
                    voxel_size=voxel_size,
                    iou_threshold=iou_threshold
                )
                detector_scene_results.append(scene_result)
                
                print(f"  {gt_scene_id}:")
                print(f"    GT instances: {scene_result['num_gt_instances']}, "
                      f"Pred instances: {scene_result['num_pred_instances']}")
                print(f"    mAP@0.5: {scene_result['map_0.5']:.4f}")
                print(f"    Classes evaluated: {scene_result['num_classes_evaluated']}")
                
            except Exception as e:
                print(f"  [ERROR] Failed to evaluate {gt_scene_id}: {e}")
                import traceback
                traceback.print_exc()
                continue
        
        # Aggregate results for this detector
        if detector_scene_results:
            aggregated = aggregate_map_results(detector_scene_results)
            
            print(f"\n  {detector.upper()} OVERALL RESULTS:")
            print(f"    Scenes evaluated: {aggregated['num_scenes']}")
            print(f"    Total GT instances: {aggregated['total_gt_instances']}")
            print(f"    Total Pred instances: {aggregated['total_pred_instances']}")
            print(f"    Mean mAP@0.5: {aggregated['mean_map_0.5']:.4f}")
            
            all_results[detector] = {
                'overall_metrics': aggregated,
                'per_scene_results': detector_scene_results
            }
        else:
            print(f"  [WARNING] No scenes evaluated for {detector}")
    
    # Save JSON results
    output = {}
    for detector in ['detectron', 'talos', 'yoloe']:
        if detector in all_results:
            output[detector] = all_results[detector]
    
    with open(output_json, 'w') as f:
        json.dump(output, f, indent=2)
    
    # Write text report
    write_text_report(all_results, gt_scenes, output_txt, dataset)
    
    print(f"\n{'='*60}")
    print(f"Evaluation complete!")
    print(f"Results saved to:")
    print(f"  - JSON: {output_json}")
    print(f"  - TXT:  {output_txt}")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()
