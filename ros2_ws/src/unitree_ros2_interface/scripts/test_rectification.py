#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
test_rectification.py

Load two CameraInfo YAML config files (left/right), apply stereo rectification
to a pair of raw frames, and display them in a single window with horizontal
epipolar lines drawn to visually verify alignment.

Usage:
    python3 test_rectification.py \
        --config-left  front_camera_left \
        --config-right front_camera_right \
        --image-left   cam0.png \
        --image-right  cam1.png
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, Tuple

import cv2
import numpy as np
import yaml


# ============================================================================
# YAML loading
# ============================================================================

def load_camera_info(path: str) -> Dict:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"Config file not found: {p}")
    with open(p, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if data is None or not isinstance(data, dict):
        raise ValueError(f"Invalid YAML in {p}")
    return data


def extract_matrices(info: Dict) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, str, int, int]:
    """Return (K, D, R, P, distortion_model, width, height) from a CameraInfo dict."""
    w = int(info["image_width"])
    h = int(info["image_height"])
    dist_model = str(info["distortion_model"])

    K = np.array(info["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    D = np.array(info["distortion_coefficients"]["data"], dtype=np.float64)
    R = np.array(info["rectification_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    P = np.array(info["projection_matrix"]["data"], dtype=np.float64).reshape(3, 4)

    return K, D, R, P, dist_model, w, h


# ============================================================================
# Rectification maps
# ============================================================================

def build_rectification_maps(
    K: np.ndarray,
    D: np.ndarray,
    R: np.ndarray,
    P: np.ndarray,
    dist_model: str,
    size: Tuple[int, int],
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute (map1, map2) for cv2.remap."""
    new_K = P[:, :3]

    if dist_model == "equidistant":
        D_cv = D.reshape(4, 1)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D_cv, R, new_K, size, cv2.CV_16SC2)
    elif dist_model == "plumb_bob":
        D_cv = D.reshape(-1, 1)
        map1, map2 = cv2.initUndistortRectifyMap(K, D_cv, R, new_K, size, cv2.CV_16SC2)
    else:
        raise ValueError(f"Unsupported distortion model: {dist_model}")

    return map1, map2


# ============================================================================
# Visualization
# ============================================================================

def draw_epipolar_lines(img: np.ndarray, n_lines: int = 20, color: Tuple[int, ...] = (0, 255, 0)) -> np.ndarray:
    """Draw horizontal lines across the image for epipolar verification."""
    out = img.copy()
    h = out.shape[0]
    step = h // (n_lines + 1)
    for i in range(1, n_lines + 1):
        y = i * step
        cv2.line(out, (0, y), (out.shape[1] - 1, y), color, 1, cv2.LINE_AA)
    return out


def compose_visualization(
    raw_left: np.ndarray,
    raw_right: np.ndarray,
    rect_left: np.ndarray,
    rect_right: np.ndarray,
    scale: float,
) -> np.ndarray:
    """
    Build a 2x2 grid:
        [ raw_left   | raw_right   ]
        [ rect_left  | rect_right  ]
    with epipolar lines on the rectified row.
    """
    # Draw epipolar lines on rectified pair
    rect_left_vis = draw_epipolar_lines(rect_left)
    rect_right_vis = draw_epipolar_lines(rect_right)

    # Labels
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.7
    thickness = 2
    label_color = (0, 200, 255)

    def put_label(img: np.ndarray, text: str) -> np.ndarray:
        out = img.copy()
        cv2.putText(out, text, (10, 30), font, font_scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
        cv2.putText(out, text, (10, 30), font, font_scale, label_color, thickness, cv2.LINE_AA)
        return out

    raw_left_vis = put_label(raw_left, "Raw Left (cam0)")
    raw_right_vis = put_label(raw_right, "Raw Right (cam1)")
    rect_left_vis = put_label(rect_left_vis, "Rectified Left")
    rect_right_vis = put_label(rect_right_vis, "Rectified Right")

    top_row = np.hstack([raw_left_vis, raw_right_vis])
    bot_row = np.hstack([rect_left_vis, rect_right_vis])
    canvas = np.vstack([top_row, bot_row])

    if scale != 1.0:
        new_w = int(canvas.shape[1] * scale)
        new_h = int(canvas.shape[0] * scale)
        canvas = cv2.resize(canvas, (new_w, new_h), interpolation=cv2.INTER_AREA)

    return canvas


# ============================================================================
# CLI
# ============================================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize raw vs rectified stereo frames using CameraInfo YAML configs."
    )
    parser.add_argument("--config-left", required=True, help="Left CameraInfo YAML file.")
    parser.add_argument("--config-right", required=True, help="Right CameraInfo YAML file.")
    parser.add_argument("--image-left", required=True, help="Raw left image (cam0).")
    parser.add_argument("--image-right", required=True, help="Raw right image (cam1).")
    parser.add_argument("--scale", type=float, default=0.5,
                        help="Display scale factor (default: 0.5).")
    return parser.parse_args()


# ============================================================================
# Main
# ============================================================================

def main() -> int:
    args = parse_args()

    # Load configs
    info_left = load_camera_info(args.config_left)
    info_right = load_camera_info(args.config_right)

    K_l, D_l, R_l, P_l, dist_l, w_l, h_l = extract_matrices(info_left)
    K_r, D_r, R_r, P_r, dist_r, w_r, h_r = extract_matrices(info_right)

    if dist_l != dist_r:
        raise ValueError(f"Distortion models differ: left={dist_l}, right={dist_r}")

    print(f"Distortion model : {dist_l}")
    print(f"Calibration size : {w_l} x {h_l}")

    # Load images
    img_left = cv2.imread(args.image_left)
    if img_left is None:
        raise FileNotFoundError(f"Cannot read left image: {args.image_left}")
    img_right = cv2.imread(args.image_right)
    if img_right is None:
        raise FileNotFoundError(f"Cannot read right image: {args.image_right}")

    if img_left.shape[:2] != (h_l, w_l):
        print(f"WARNING: left image size {img_left.shape[1]}x{img_left.shape[0]} "
              f"!= config size {w_l}x{h_l}")
    if img_right.shape[:2] != (h_r, w_r):
        print(f"WARNING: right image size {img_right.shape[1]}x{img_right.shape[0]} "
              f"!= config size {w_r}x{h_r}")

    # Build rectification maps
    size_l = (w_l, h_l)
    size_r = (w_r, h_r)
    map1_l, map2_l = build_rectification_maps(K_l, D_l, R_l, P_l, dist_l, size_l)
    map1_r, map2_r = build_rectification_maps(K_r, D_r, R_r, P_r, dist_r, size_r)

    # Rectify
    rect_left = cv2.remap(img_left, map1_l, map2_l, cv2.INTER_LINEAR)
    rect_right = cv2.remap(img_right, map1_r, map2_r, cv2.INTER_LINEAR)

    # Print baseline from P_right
    tx = P_r[0, 3]
    fx = P_r[0, 0]
    baseline = -tx / fx if abs(fx) > 0 else 0.0
    print(f"Baseline from P  : {baseline:.6f} m")
    print(f"Rect focal length: {fx:.4f} px")

    # Compose and display
    canvas = compose_visualization(img_left, img_right, rect_left, rect_right, args.scale)

    window_name = "Stereo Rectification Test"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.imshow(window_name, canvas)
    print("\nPress any key or close the window to exit.")

    while True:
        key = cv2.waitKey(100) & 0xFF
        if key != 255:                 # any key pressed
            break
        if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
            break                      # window closed via X button

    cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
