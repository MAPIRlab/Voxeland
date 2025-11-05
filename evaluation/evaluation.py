#!/usr/bin/env python3
"""
Evaluation Script for Voxeland
Compares Voxeland detector outputs (detectron, talos, yoloe) against ground truth.
Supports both ScanNet and SceneNN datasets.
Computes Precision, Recall, and F1-score at IoU thresholds of 0.25 and 0.5.

Usage:
    python3 evaluation.py scannet
    python3 evaluation.py scenenn
"""

import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Set
import numpy as np
from collections import defaultdict


def compute_iou_3d(bbox1: Dict, bbox2: Dict) -> float:
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


def check_semantic_match(gt_class: str, gt_synonyms: List[str], pred_class: str) -> bool:
    """
    Check if predicted class matches ground truth class or any of its synonyms.
    Case-insensitive comparison.
    
    Args:
        gt_class: Ground truth class name
        gt_synonyms: List of synonym names for the ground truth class
        pred_class: Predicted class name
    
    Returns:
        True if there's a semantic match, False otherwise
    """
    pred_class_lower = pred_class.lower().strip()
    
    # Check direct match with class name
    if pred_class_lower == gt_class.lower().strip():
        return True
    
    # Check match with synonyms
    for synonym in gt_synonyms:
        if pred_class_lower == synonym.lower().strip():
            return True
    
    return False


def match_instances(gt_instances: List[Dict], pred_instances: List[Dict], iou_threshold: float) -> Tuple[int, int, int, List[Dict]]:
    """
    Match predicted instances to ground truth instances based on semantic category and IoU.
    
    Args:
        gt_instances: List of ground truth instances
        pred_instances: List of predicted instances
        iou_threshold: IoU threshold for considering a match (0.25 or 0.5)
    
    Returns:
        Tuple of (true_positives, false_positives, false_negatives, match_details)
    """
    # New approach: build all candidate pairs (gt_idx, pred_idx, iou)
    # where semantic match holds and iou >= threshold. Then perform
    # a global greedy matching by IoU descending to ensure results are
    # invariant to the input ordering.
    match_details: List[Dict] = []

    # Filter out "unknown" class from predictions and keep indices mapping
    valid_preds = [(pidx, p) for pidx, p in enumerate(pred_instances) if p['class_name'].lower() != 'unknown']

    candidates: List[Tuple[int, int, float]] = []
    for p_local_idx, (pidx, pred) in enumerate(valid_preds):
        for gt_idx, gt in enumerate(gt_instances):
            # semantic match
            if not check_semantic_match(gt['class_name'], gt.get('synonyms', []), pred['class_name']):
                continue
            iou = compute_iou_3d(gt['aabb'], pred['aabb'])
            if iou >= iou_threshold:
                candidates.append((gt_idx, pidx, iou))

    # Sort candidates by IoU descending for greedy matching
    candidates.sort(key=lambda x: x[2], reverse=True)

    matched_gt: Set[int] = set()
    matched_pred: Set[int] = set()

    for gt_idx, pidx, iou in candidates:
        if gt_idx in matched_gt or pidx in matched_pred:
            continue
        # assign match
        matched_gt.add(gt_idx)
        matched_pred.add(pidx)
        match_details.append({
            'gt_instance_id': gt_instances[gt_idx]['instance_id'],
            'pred_instance_id': pred_instances[pidx]['instance_id'],
            'gt_class': gt_instances[gt_idx]['class_name'],
            'pred_class': pred_instances[pidx]['class_name'],
            'iou': iou,
            'match_type': 'TP'
        })

    true_positives = len(matched_gt)
    false_positives = len(valid_preds) - len(matched_pred)
    false_negatives = len(gt_instances) - len(matched_gt)

    # Optionally append FP and FN summary entries (kept minimal)
    for pidx, pred in valid_preds:
        if pidx not in matched_pred:
            match_details.append({
                'gt_instance_id': None,
                'pred_instance_id': pred['instance_id'],
                'gt_class': None,
                'pred_class': pred['class_name'],
                'iou': 0.0,
                'match_type': 'FP'
            })

    for gt_idx, gt in enumerate(gt_instances):
        if gt_idx not in matched_gt:
            match_details.append({
                'gt_instance_id': gt['instance_id'],
                'pred_instance_id': None,
                'gt_class': gt['class_name'],
                'pred_class': None,
                'iou': 0.0,
                'match_type': 'FN'
            })

    return true_positives, false_positives, false_negatives, match_details


def compute_metrics(tp: int, fp: int, fn: int) -> Dict[str, float]:
    """
    Compute Precision, Recall, and F1-score from TP, FP, FN counts.
    
    Args:
        tp: True positives
        fp: False positives
        fn: False negatives
    
    Returns:
        Dictionary with precision, recall, and f1_score
    """
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0

    # Return raw (unrounded) floats. Formatting/rounding should be
    # done at presentation time to avoid accumulation rounding errors.
    return {
        'precision': precision,
        'recall': recall,
        'f1_score': f1_score,
        'true_positives': tp,
        'false_positives': fp,
        'false_negatives': fn
    }


def evaluate_scene(scene_id: str, gt_data: Dict, pred_data: Dict, detector: str) -> Dict:
    """
    Evaluate a single scene for a specific detector.
    
    Args:
        scene_id: Scene identifier
        gt_data: Ground truth data
        pred_data: Predicted data
        detector: Detector name (detectron, talos, yoloe)
    
    Returns:
        Dictionary with evaluation results for this scene
    """
    gt_instances = gt_data['instances']
    pred_instances = pred_data['instances']
    
    # Evaluate at IoU 0.25
    tp_25, fp_25, fn_25, _ = match_instances(gt_instances, pred_instances, 0.25)
    metrics_25 = compute_metrics(tp_25, fp_25, fn_25)
    
    # Evaluate at IoU 0.5
    tp_50, fp_50, fn_50, _ = match_instances(gt_instances, pred_instances, 0.5)
    metrics_50 = compute_metrics(tp_50, fp_50, fn_50)
    
    return {
        'scene_id': scene_id,
        'num_gt_instances': len(gt_instances),
        'num_pred_instances': len([p for p in pred_instances if p['class_name'].lower() != 'unknown']),
        'metrics_iou_0.25': metrics_25,
        'metrics_iou_0.5': metrics_50
    }


def aggregate_metrics(scene_results: List[Dict]) -> Dict:
    """
    Aggregate metrics across all scenes for a detector.
    
    Args:
        scene_results: List of per-scene evaluation results
    
    Returns:
        Aggregated metrics
    """
    # Aggregate counts for IoU 0.25
    total_tp_25 = sum(r['metrics_iou_0.25']['true_positives'] for r in scene_results)
    total_fp_25 = sum(r['metrics_iou_0.25']['false_positives'] for r in scene_results)
    total_fn_25 = sum(r['metrics_iou_0.25']['false_negatives'] for r in scene_results)
    
    # Aggregate counts for IoU 0.5
    total_tp_50 = sum(r['metrics_iou_0.5']['true_positives'] for r in scene_results)
    total_fp_50 = sum(r['metrics_iou_0.5']['false_positives'] for r in scene_results)
    total_fn_50 = sum(r['metrics_iou_0.5']['false_negatives'] for r in scene_results)
    
    # Compute overall metrics
    overall_metrics_25 = compute_metrics(total_tp_25, total_fp_25, total_fn_25)
    overall_metrics_50 = compute_metrics(total_tp_50, total_fp_50, total_fn_50)

    return {
        'num_scenes': len(scene_results),
        'total_gt_instances': sum(r['num_gt_instances'] for r in scene_results),
        'total_pred_instances': sum(r['num_pred_instances'] for r in scene_results),
        'metrics_iou_0.25': overall_metrics_25,
        'metrics_iou_0.5': overall_metrics_50
    }


def write_text_report(all_results: Dict, scenes: List[str], output_file: Path, dataset_name: str):
    """
    Write a human-readable text report with all metrics and detector ranking.
    Organized in two main blocks: IoU@0.25 and IoU@0.5
    """
    with open(output_file, 'w') as f:
        f.write("="*80 + "\n")
        f.write(f"{dataset_name.upper()} EVALUATION RESULTS - VOXELAND\n")
        f.write("="*80 + "\n\n")
        
        # ========================================================================
        # BLOCK 1: IoU @ 0.25
        # ========================================================================
        f.write("#" * 80 + "\n")
        f.write("###" + " " * 25 + "METRICS @ IoU 0.25" + " " * 25 + "###\n")
        f.write("#" * 80 + "\n\n")
        
        # Detector ranking @ 0.25
        f.write("DETECTOR RANKING @ IoU 0.25\n")
        f.write("-"*80 + "\n")
        rankings_25 = []
        for detector in ['detectron', 'talos', 'yoloe']:
            if detector in all_results:
                overall = all_results[detector]['overall_metrics']
                f1_25 = overall['metrics_iou_0.25']['f1_score']
                rankings_25.append((detector, f1_25))
        
        rankings_25.sort(key=lambda x: x[1], reverse=True)
        
        f.write(f"{'Rank':<6} {'Detector':<12} {'F1-Score':>12} {'Precision':>12} {'Recall':>12}\n")
        f.write("-"*80 + "\n")
        for rank, (detector, f1) in enumerate(rankings_25, 1):
            overall = all_results[detector]['overall_metrics']
            m25 = overall['metrics_iou_0.25']
            f.write(f"{rank:<6} {detector.upper():<12} {m25['f1_score']:>12.4f} {m25['precision']:>12.4f} {m25['recall']:>12.4f}\n")
        f.write("-"*80 + "\n\n")
        
        # Overall results for each detector @ 0.25
        for detector in ['detectron', 'talos', 'yoloe']:
            if detector not in all_results:
                continue
                
            overall = all_results[detector]['overall_metrics']
            
            f.write("="*80 + "\n")
            f.write(f"{detector.upper()} - OVERALL RESULTS @ IoU 0.25\n")
            f.write("="*80 + "\n")
            f.write(f"Scenes Evaluated: {overall['num_scenes']}\n")
            f.write(f"Total Ground Truth Instances: {overall['total_gt_instances']}\n")
            f.write(f"Total Predicted Instances: {overall['total_pred_instances']}\n\n")
            
            m25 = overall['metrics_iou_0.25']
            f.write(f"  Precision:        {m25['precision']:.4f}\n")
            f.write(f"  Recall:           {m25['recall']:.4f}\n")
            f.write(f"  F1-Score:         {m25['f1_score']:.4f}\n")
            f.write(f"  True Positives:   {m25['true_positives']}\n")
            f.write(f"  False Positives:  {m25['false_positives']}\n")
            f.write(f"  False Negatives:  {m25['false_negatives']}\n\n")
        
        # Per-scene comparison @ 0.25
        f.write("="*80 + "\n")
        f.write("PER-SCENE COMPARISON @ IoU 0.25\n")
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
                    f1 = scene_results[0]['metrics_iou_0.25']['f1_score']
                    scene_scores[detector] = f1
                else:
                    scene_scores[detector] = None
            
            # Find best detector(s) with tie handling
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
        
        f.write("-"*80 + "\n\n\n")
        
        # ========================================================================
        # BLOCK 2: IoU @ 0.5
        # ========================================================================
        f.write("#" * 80 + "\n")
        f.write("###" + " " * 25 + "METRICS @ IoU 0.50" + " " * 25 + "###\n")
        f.write("#" * 80 + "\n\n")
        
        # Detector ranking @ 0.5
        f.write("DETECTOR RANKING @ IoU 0.50\n")
        f.write("-"*80 + "\n")
        rankings_50 = []
        for detector in ['detectron', 'talos', 'yoloe']:
            if detector in all_results:
                overall = all_results[detector]['overall_metrics']
                f1_50 = overall['metrics_iou_0.5']['f1_score']
                rankings_50.append((detector, f1_50))
        
        rankings_50.sort(key=lambda x: x[1], reverse=True)
        
        f.write(f"{'Rank':<6} {'Detector':<12} {'F1-Score':>12} {'Precision':>12} {'Recall':>12}\n")
        f.write("-"*80 + "\n")
        for rank, (detector, f1) in enumerate(rankings_50, 1):
            overall = all_results[detector]['overall_metrics']
            m50 = overall['metrics_iou_0.5']
            f.write(f"{rank:<6} {detector.upper():<12} {m50['f1_score']:>12.4f} {m50['precision']:>12.4f} {m50['recall']:>12.4f}\n")
        f.write("-"*80 + "\n\n")
        
        # Overall results for each detector @ 0.5
        for detector in ['detectron', 'talos', 'yoloe']:
            if detector not in all_results:
                continue
                
            overall = all_results[detector]['overall_metrics']
            
            f.write("="*80 + "\n")
            f.write(f"{detector.upper()} - OVERALL RESULTS @ IoU 0.50\n")
            f.write("="*80 + "\n")
            f.write(f"Scenes Evaluated: {overall['num_scenes']}\n")
            f.write(f"Total Ground Truth Instances: {overall['total_gt_instances']}\n")
            f.write(f"Total Predicted Instances: {overall['total_pred_instances']}\n\n")
            
            m50 = overall['metrics_iou_0.5']
            f.write(f"  Precision:        {m50['precision']:.4f}\n")
            f.write(f"  Recall:           {m50['recall']:.4f}\n")
            f.write(f"  F1-Score:         {m50['f1_score']:.4f}\n")
            f.write(f"  True Positives:   {m50['true_positives']}\n")
            f.write(f"  False Positives:  {m50['false_positives']}\n")
            f.write(f"  False Negatives:  {m50['false_negatives']}\n\n")
        
        # Per-scene comparison @ 0.5
        f.write("="*80 + "\n")
        f.write("PER-SCENE COMPARISON @ IoU 0.50\n")
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
                    f1 = scene_results[0]['metrics_iou_0.5']['f1_score']
                    scene_scores[detector] = f1
                else:
                    scene_scores[detector] = None
            
            # Find best detector(s) with tie handling
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


def get_prediction_filename(scene_id: str, detector: str, dataset: str) -> str:
    """
    Get the prediction filename based on dataset and scene_id.
    
    Args:
        scene_id: Scene identifier (e.g., "scene0000_01" for ScanNet, "011" for SceneNN)
        detector: Detector name (detectron, talos, yoloe)
        dataset: Dataset name (scannet or scenenn)
    
    Returns:
        Prediction filename
    """
    if dataset == 'scannet':
        # ScanNet format: voxeland_semantic_map_detectron_s0000_01.json
        scene_suffix = scene_id.replace('scene', '')
        return f"voxeland_semantic_map_{detector}_s{scene_suffix}.json"
    else:  # scenenn
        # SceneNN format: voxeland_semantic_map_detectron_s011.json
        return f"voxeland_semantic_map_{detector}_s{scene_id}.json"


def main():
    """
    Main evaluation function.
    """
    # Check command line arguments
    if len(sys.argv) != 2 or sys.argv[1].lower() not in ['scannet', 'scenenn']:
        print("Usage: python3 evaluation.py <dataset>")
        print("  dataset: 'scannet' or 'scenenn'")
        sys.exit(1)
    
    dataset = sys.argv[1].lower()
    
    # Define paths
    script_dir = Path(__file__).parent
    gt_dir = script_dir / f'{dataset}_groundtruth'
    pred_dir = script_dir / 'voxeland_output'
    output_json = script_dir / f'{dataset}_evaluation_results.json'
    output_txt = script_dir / f'{dataset}_evaluation_results.txt'
    
    # Check if ground truth directory exists
    if not gt_dir.exists():
        print(f"ERROR: Ground truth directory not found: {gt_dir}")
        sys.exit(1)
    
    if not pred_dir.exists():
        print(f"ERROR: Prediction directory not found: {pred_dir}")
        sys.exit(1)
    
    # Get list of scenes
    scenes = sorted([d.name for d in gt_dir.iterdir() if d.is_dir()])
    detectors = ['detectron', 'talos', 'yoloe']
    
    print(f"Starting {dataset.upper()} evaluation...")
    print(f"Found {len(scenes)} scenes: {scenes}")
    print(f"Evaluating {len(detectors)} detectors: {detectors}")
    print()
    
    # Results structure
    all_results = {}
    
    # Evaluate each detector
    for detector in detectors:
        print(f"\n{'='*60}")
        print(f"Evaluating detector: {detector.upper()}")
        print(f"{'='*60}")
        
        detector_scene_results = []
        
        for scene_id in scenes:
            # Load ground truth
            gt_file = gt_dir / scene_id / f"{scene_id}_gt_instances_aabb_synonyms.json"
            if not gt_file.exists():
                print(f"  [WARNING] Ground truth not found for {scene_id}, skipping...")
                continue
            
            with open(gt_file, 'r') as f:
                gt_data = json.load(f)
            
            # Load prediction
            pred_filename = get_prediction_filename(scene_id, detector, dataset)
            pred_file = pred_dir / scene_id / pred_filename
            if not pred_file.exists():
                print(f"  [WARNING] Prediction not found for {scene_id} with {detector}, skipping...")
                continue
            
            with open(pred_file, 'r') as f:
                pred_data = json.load(f)
            
            # Evaluate scene
            scene_result = evaluate_scene(scene_id, gt_data, pred_data, detector)
            detector_scene_results.append(scene_result)
            
            print(f"  {scene_id}:")
            print(f"    GT instances: {scene_result['num_gt_instances']}, "
                  f"Pred instances: {scene_result['num_pred_instances']}")
            print(f"    IoU@0.25 - P: {scene_result['metrics_iou_0.25']['precision']:.3f}, "
                  f"R: {scene_result['metrics_iou_0.25']['recall']:.3f}, "
                  f"F1: {scene_result['metrics_iou_0.25']['f1_score']:.3f}")
            print(f"    IoU@0.50 - P: {scene_result['metrics_iou_0.5']['precision']:.3f}, "
                  f"R: {scene_result['metrics_iou_0.5']['recall']:.3f}, "
                  f"F1: {scene_result['metrics_iou_0.5']['f1_score']:.3f}")
        
        # Aggregate results for this detector
        if detector_scene_results:
            aggregated = aggregate_metrics(detector_scene_results)
            
            print(f"\n  {detector.upper()} OVERALL RESULTS:")
            print(f"    Scenes evaluated: {aggregated['num_scenes']}")
            print(f"    Total GT instances: {aggregated['total_gt_instances']}")
            print(f"    Total Pred instances: {aggregated['total_pred_instances']}")
            print(f"    IoU@0.25 - P: {aggregated['metrics_iou_0.25']['precision']:.3f}, "
                  f"R: {aggregated['metrics_iou_0.25']['recall']:.3f}, "
                  f"F1: {aggregated['metrics_iou_0.25']['f1_score']:.3f}")
            print(f"    IoU@0.50 - P: {aggregated['metrics_iou_0.5']['precision']:.3f}, "
                  f"R: {aggregated['metrics_iou_0.5']['recall']:.3f}, "
                  f"F1: {aggregated['metrics_iou_0.5']['f1_score']:.3f}")
            
            # Simplify scene results - remove match_details
            simplified_scene_results = []
            for scene_result in detector_scene_results:
                simplified_scene_results.append({
                    'scene_id': scene_result['scene_id'],
                    'num_gt_instances': scene_result['num_gt_instances'],
                    'num_pred_instances': scene_result['num_pred_instances'],
                    'metrics_iou_0.25': scene_result['metrics_iou_0.25'],
                    'metrics_iou_0.5': scene_result['metrics_iou_0.5']
                })
            
            all_results[detector] = {
                'overall_metrics': aggregated,
                'per_scene_results': simplified_scene_results
            }
        else:
            print(f"  [WARNING] No scenes evaluated for {detector}")
    
    # Save simplified JSON with only metrics
    output = {}
    for detector in ['detectron', 'talos', 'yoloe']:
        if detector in all_results:
            output[detector] = all_results[detector]
    
    with open(output_json, 'w') as f:
        json.dump(output, f, indent=2)
    
    # Write text report
    write_text_report(all_results, scenes, output_txt, dataset)
    
    print(f"\n{'='*60}")
    print(f"Evaluation complete!")
    print(f"Results saved to:")
    print(f"  - JSON: {output_json}")
    print(f"  - TXT:  {output_txt}")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()
