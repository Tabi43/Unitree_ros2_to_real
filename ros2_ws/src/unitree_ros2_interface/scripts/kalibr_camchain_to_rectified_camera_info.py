#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Convert a stereo Kalibr camchain.yaml into ROS-style CameraInfo YAML files and
OpenCV rectification maps.

Supported raw inputs
--------------------
- pinhole + equidistant/equi      -> OpenCV fisheye backend
- pinhole + radtan/plumb_bob      -> OpenCV standard pinhole backend

Experimental mode
-----------------
For pinhole + equidistant inputs, you may request an approximate "Unitree-like"
pinhole/plumb_bob export by fitting a surrogate plumb_bob raw model to the
central part of the fisheye mapping. This is an approximation, not a new
calibration. It can be useful if a downstream stack expects standard
plumb_bob/pinhole CameraInfo.

ROS CameraInfo semantics
------------------------
The exported CameraInfo always follows the ROS convention:
- K : raw intrinsics for the raw image stream
- D : raw distortion coefficients of the chosen runtime model
- R : rectification rotation
- P : projection matrix of the rectified image

If you enable the experimental surrogate plumb_bob mode, K and D are the fitted
surrogate pinhole/plumb_bob parameters, not the original equidistant ones.

Kalibr extrinsics
-----------------
Kalibr stores cam1.T_cn_cnm1 as the transform from cam0 coordinates to cam1
coordinates. This script uses that convention directly.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, Optional, Tuple, Any

import cv2
import numpy as np
import yaml

try:
    from scipy.optimize import least_squares
except Exception:  # pragma: no cover
    least_squares = None


@dataclass
class RawCamera:
    name: str
    model: str                # "pinhole"
    distortion: str           # "equidistant" / "equi" / "radtan" / "plumb_bob"
    width: int
    height: int
    K: np.ndarray             # 3x3
    D: np.ndarray             # Nx1 raw distortion vector


@dataclass
class RuntimeCamera:
    width: int
    height: int
    ros_distortion_model: str
    K: np.ndarray             # raw intrinsics of runtime model
    D: np.ndarray             # raw distortion coeffs of runtime model


@dataclass
class StereoRuntimeModel:
    backend: str              # "fisheye" or "standard"
    mode: str                 # "auto_raw_model" or "surrogate_plumb_bob"
    cam0: RuntimeCamera
    cam1: RuntimeCamera
    metadata: Dict[str, Any]


@dataclass
class RectificationResult:
    R1: np.ndarray
    R2: np.ndarray
    P1: np.ndarray
    P2: np.ndarray
    Q: np.ndarray
    map1_left: np.ndarray
    map2_left: np.ndarray
    map1_right: np.ndarray
    map2_right: np.ndarray
    roi1: Optional[Tuple[int, int, int, int]]
    roi2: Optional[Tuple[int, int, int, int]]


def _as_contig(a: np.ndarray) -> np.ndarray:
    return np.ascontiguousarray(np.asarray(a, dtype=np.float64))


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError("Input YAML must contain a top-level mapping.")
    return data


def _intrinsics_to_K(vals) -> np.ndarray:
    if len(vals) != 4:
        raise ValueError(f"Pinhole intrinsics must have length 4, got {len(vals)}.")
    fx, fy, cx, cy = map(float, vals)
    return np.array([[fx, 0.0, cx],
                     [0.0, fy, cy],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def _parse_camera(node_name: str, node: dict) -> RawCamera:
    for key in ["camera_model", "distortion_model", "intrinsics", "distortion_coeffs", "resolution"]:
        if key not in node:
            raise ValueError(f"{node_name} is missing required field '{key}'.")
    model = str(node["camera_model"]).strip().lower()
    distortion = str(node["distortion_model"]).strip().lower()
    if model != "pinhole":
        raise ValueError(f"{node_name}: only 'pinhole' camera_model is supported, got '{model}'.")
    width, height = map(int, node["resolution"])
    K = _intrinsics_to_K(node["intrinsics"])
    D = np.asarray(node["distortion_coeffs"], dtype=np.float64).reshape(-1, 1)
    return RawCamera(
        name=node_name,
        model=model,
        distortion=distortion,
        width=width,
        height=height,
        K=K,
        D=D,
    )


def _parse_camchain(path: Path):
    data = _load_yaml(path)
    if "cam0" not in data or "cam1" not in data:
        raise ValueError("Input camchain must contain cam0 and cam1.")
    cam0 = _parse_camera("cam0", data["cam0"])
    cam1 = _parse_camera("cam1", data["cam1"])
    if cam0.width != cam1.width or cam0.height != cam1.height:
        raise ValueError("cam0 and cam1 must have the same resolution.")
    if cam0.distortion != cam1.distortion:
        raise ValueError("cam0 and cam1 must use the same distortion model family.")

    T = np.asarray(data["cam1"].get("T_cn_cnm1", None), dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError("cam1.T_cn_cnm1 must be a 4x4 homogeneous transform.")
    R_10 = T[:3, :3].copy()
    t_10 = T[:3, 3].reshape(3, 1).copy()

    return cam0, cam1, R_10, t_10


def _normalize_distortion_name(name: str) -> str:
    name = name.strip().lower()
    aliases = {
        "equi": "equidistant",
        "equidistant": "equidistant",
        "radtan": "radtan",
        "plumb_bob": "radtan",
    }
    if name not in aliases:
        raise ValueError(f"Unsupported distortion model '{name}'.")
    return aliases[name]


def _brown_project(xy: np.ndarray, params: np.ndarray) -> np.ndarray:
    fx, fy, cx, cy, k1, k2, p1, p2, k3 = params
    x = xy[:, 0]
    y = xy[:, 1]
    r2 = x * x + y * y
    r4 = r2 * r2
    r6 = r4 * r2
    radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6
    x_dist = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    y_dist = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    u = fx * x_dist + cx
    v = fy * y_dist + cy
    return np.column_stack([u, v])


def _sample_fisheye_pixels_for_surrogate_fit(K: np.ndarray,
                                             D: np.ndarray,
                                             size: Tuple[int, int],
                                             radius_ratio: float,
                                             grid_n: int):
    width, height = size
    cx, cy = float(K[0, 2]), float(K[1, 2])
    rmax = float(radius_ratio) * min(cx, cy, width - 1 - cx, height - 1 - cy)
    xs = np.linspace(cx - rmax, cx + rmax, grid_n)
    ys = np.linspace(cy - rmax, cy + rmax, grid_n)

    pts = np.array(
        [[x, y] for y in ys for x in xs if (x - cx) ** 2 + (y - cy) ** 2 <= rmax ** 2],
        dtype=np.float64,
    ).reshape(-1, 1, 2)

    und = cv2.fisheye.undistortPoints(
        _as_contig(pts),
        _as_contig(K),
        _as_contig(D),
        R=np.eye(3, dtype=np.float64),
        P=np.eye(3, dtype=np.float64),
    )
    xy = und.reshape(-1, 2)
    pix = pts.reshape(-1, 2)
    return xy, pix, rmax


def _fit_surrogate_plumb_bob(raw_cam: RawCamera,
                             radius_ratio: float,
                             grid_n: int) -> Tuple[RuntimeCamera, Dict[str, Any]]:
    if least_squares is None:
        raise RuntimeError("scipy is required for surrogate plumb_bob fitting but is not available.")

    size = (raw_cam.width, raw_cam.height)
    xy, pix, fit_radius_px = _sample_fisheye_pixels_for_surrogate_fit(
        raw_cam.K, raw_cam.D, size=size, radius_ratio=radius_ratio, grid_n=grid_n
    )

    p0 = np.array([
        raw_cam.K[0, 0], raw_cam.K[1, 1], raw_cam.K[0, 2], raw_cam.K[1, 2],
        0.0, 0.0, 0.0, 0.0, 0.0
    ], dtype=np.float64)

    lb = np.array([50.0, 50.0, 0.0, 0.0, -10.0, -10.0, -1.0, -1.0, -10.0], dtype=np.float64)
    ub = np.array([2000.0, 2000.0, raw_cam.width, raw_cam.height, 10.0, 10.0, 1.0, 1.0, 10.0], dtype=np.float64)

    def residuals(p: np.ndarray) -> np.ndarray:
        pred = _brown_project(xy, p)
        return (pred - pix).ravel()

    result = least_squares(
        residuals,
        p0,
        bounds=(lb, ub),
        loss="soft_l1",
        f_scale=1.0,
        max_nfev=500,
    )
    p = result.x
    rmse_px = float(np.sqrt(np.mean(residuals(p) ** 2)))

    K_fit = np.array([[p[0], 0.0, p[2]],
                      [0.0, p[1], p[3]],
                      [0.0, 0.0, 1.0]], dtype=np.float64)
    D_fit = np.array([p[4], p[5], p[6], p[7], p[8]], dtype=np.float64).reshape(-1, 1)

    runtime_cam = RuntimeCamera(
        width=raw_cam.width,
        height=raw_cam.height,
        ros_distortion_model="plumb_bob",
        K=K_fit,
        D=D_fit,
    )
    meta = {
        "fit_radius_ratio": float(radius_ratio),
        "fit_radius_px": float(fit_radius_px),
        "fit_grid_n": int(grid_n),
        "fit_rmse_px": rmse_px,
        "fitted_params": {
            "fx": float(p[0]), "fy": float(p[1]), "cx": float(p[2]), "cy": float(p[3]),
            "k1": float(p[4]), "k2": float(p[5]), "p1": float(p[6]), "p2": float(p[7]), "k3": float(p[8]),
        },
    }
    return runtime_cam, meta


def _build_runtime_model(raw_cam0: RawCamera,
                         raw_cam1: RawCamera,
                         mode: str,
                         surrogate_fit_radius_ratio: float,
                         surrogate_fit_grid_n: int) -> StereoRuntimeModel:
    distortion = _normalize_distortion_name(raw_cam0.distortion)

    if mode == "auto":
        if distortion == "equidistant":
            rt0 = RuntimeCamera(raw_cam0.width, raw_cam0.height, "equidistant", raw_cam0.K.copy(), raw_cam0.D.copy())
            rt1 = RuntimeCamera(raw_cam1.width, raw_cam1.height, "equidistant", raw_cam1.K.copy(), raw_cam1.D.copy())
            return StereoRuntimeModel(
                backend="fisheye",
                mode="auto_raw_model",
                cam0=rt0,
                cam1=rt1,
                metadata={"source_distortion_model": distortion},
            )
        elif distortion == "radtan":
            def radtan_runtime(raw: RawCamera) -> RuntimeCamera:
                # ROS plumb_bob convention is k1,k2,p1,p2,k3. Kalibr radtan is typically 4 terms.
                d = raw.D.reshape(-1)
                if d.size == 4:
                    d = np.array([d[0], d[1], d[2], d[3], 0.0], dtype=np.float64)
                elif d.size == 5:
                    d = d.astype(np.float64)
                else:
                    raise ValueError(f"Unsupported radtan coefficient length {d.size}; expected 4 or 5.")
                return RuntimeCamera(raw.width, raw.height, "plumb_bob", raw.K.copy(), d.reshape(-1, 1))

            return StereoRuntimeModel(
                backend="standard",
                mode="auto_raw_model",
                cam0=radtan_runtime(raw_cam0),
                cam1=radtan_runtime(raw_cam1),
                metadata={"source_distortion_model": distortion},
            )
        else:
            raise ValueError(f"Unhandled raw distortion model '{distortion}'.")

    if mode == "surrogate_plumb_bob":
        if distortion != "equidistant":
            raise ValueError("surrogate_plumb_bob mode is only valid for equidistant/equi raw inputs.")
        rt0, meta0 = _fit_surrogate_plumb_bob(raw_cam0, surrogate_fit_radius_ratio, surrogate_fit_grid_n)
        rt1, meta1 = _fit_surrogate_plumb_bob(raw_cam1, surrogate_fit_radius_ratio, surrogate_fit_grid_n)
        return StereoRuntimeModel(
            backend="standard",
            mode="surrogate_plumb_bob",
            cam0=rt0,
            cam1=rt1,
            metadata={
                "source_distortion_model": distortion,
                "surrogate_cam0": meta0,
                "surrogate_cam1": meta1,
            },
        )

    raise ValueError(f"Unknown mode '{mode}'.")


def _rectify(runtime: StereoRuntimeModel,
             R_10: np.ndarray,
             t_10: np.ndarray,
             size: Tuple[int, int],
             new_size: Tuple[int, int],
             zero_disparity: bool,
             balance: float,
             fov_scale: float,
             alpha: float,
             map_type: str) -> RectificationResult:
    width, height = new_size
    cv_map_type = cv2.CV_16SC2 if map_type == "16sc2" else cv2.CV_32FC1

    if runtime.backend == "fisheye":
        flags = cv2.fisheye.CALIB_ZERO_DISPARITY if zero_disparity else 0
        R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
            _as_contig(runtime.cam0.K),
            _as_contig(runtime.cam0.D),
            _as_contig(runtime.cam1.K),
            _as_contig(runtime.cam1.D),
            size,
            _as_contig(R_10),
            _as_contig(t_10),
            flags=flags,
            newImageSize=(width, height),
            balance=float(balance),
            fov_scale=float(fov_scale),
        )
        map1_left, map2_left = cv2.fisheye.initUndistortRectifyMap(
            _as_contig(runtime.cam0.K), _as_contig(runtime.cam0.D), R1, P1[:, :3], (width, height), cv_map_type
        )
        map1_right, map2_right = cv2.fisheye.initUndistortRectifyMap(
            _as_contig(runtime.cam1.K), _as_contig(runtime.cam1.D), R2, P2[:, :3], (width, height), cv_map_type
        )
        roi1 = None
        roi2 = None
    else:
        flags = cv2.CALIB_ZERO_DISPARITY if zero_disparity else 0
        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
            _as_contig(runtime.cam0.K),
            _as_contig(runtime.cam0.D),
            _as_contig(runtime.cam1.K),
            _as_contig(runtime.cam1.D),
            size,
            _as_contig(R_10),
            _as_contig(t_10),
            flags=flags,
            alpha=float(alpha),
            newImageSize=(width, height),
        )
        map1_left, map2_left = cv2.initUndistortRectifyMap(
            _as_contig(runtime.cam0.K), _as_contig(runtime.cam0.D), R1, P1[:, :3], (width, height), cv_map_type
        )
        map1_right, map2_right = cv2.initUndistortRectifyMap(
            _as_contig(runtime.cam1.K), _as_contig(runtime.cam1.D), R2, P2[:, :3], (width, height), cv_map_type
        )
        roi1 = tuple(map(int, roi1))
        roi2 = tuple(map(int, roi2))

    return RectificationResult(
        R1=np.asarray(R1, dtype=np.float64),
        R2=np.asarray(R2, dtype=np.float64),
        P1=np.asarray(P1, dtype=np.float64),
        P2=np.asarray(P2, dtype=np.float64),
        Q=np.asarray(Q, dtype=np.float64),
        map1_left=map1_left,
        map2_left=map2_left,
        map1_right=map1_right,
        map2_right=map2_right,
        roi1=roi1,
        roi2=roi2,
    )


def _flatten_row_major(a: np.ndarray):
    return [float(x) for x in np.asarray(a, dtype=np.float64).reshape(-1)]


def _camera_info_yaml(name: str, runtime_cam: RuntimeCamera, R_rect: np.ndarray, P_rect: np.ndarray) -> dict:
    return {
        "image_width": int(runtime_cam.width),
        "image_height": int(runtime_cam.height),
        "camera_name": name,
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": _flatten_row_major(runtime_cam.K),
        },
        "distortion_model": runtime_cam.ros_distortion_model,
        "distortion_coefficients": {
            "rows": int(runtime_cam.D.size),
            "cols": 1,
            "data": _flatten_row_major(runtime_cam.D),
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": _flatten_row_major(R_rect),
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": _flatten_row_major(P_rect),
        },
    }


def _save_yaml(path: Path, data: dict):
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def _projection_baseline(P: np.ndarray) -> Optional[float]:
    fx = float(P[0, 0])
    fy = float(P[1, 1])
    if abs(fx) > 1e-12 and abs(P[0, 3]) > 0:
        return float(-P[0, 3] / fx)
    if abs(fy) > 1e-12 and abs(P[1, 3]) > 0:
        return float(-P[1, 3] / fy)
    return None


def _valid_ratio_from_maps(map1: np.ndarray, map2: np.ndarray, raw_width: int, raw_height: int) -> float:
    if map1.ndim == 3 and map1.shape[2] == 2:
        x = map1[..., 0].astype(np.float64)
        y = map1[..., 1].astype(np.float64)
    else:
        x = map1.astype(np.float64)
        y = map2.astype(np.float64)
    valid = (x >= 0.0) & (x < raw_width - 1) & (y >= 0.0) & (y < raw_height - 1)
    return float(valid.mean())


def _diagnostics(raw_cam0: RawCamera,
                 raw_cam1: RawCamera,
                 runtime: StereoRuntimeModel,
                 rect: RectificationResult,
                 R_10: np.ndarray,
                 t_10: np.ndarray):
    baseline_from_T = float(np.linalg.norm(t_10))
    baseline_from_P = _projection_baseline(rect.P2)
    return {
        "input_raw_model": {
            "camera_model": raw_cam0.model,
            "distortion_model": raw_cam0.distortion,
            "image_size": [raw_cam0.width, raw_cam0.height],
        },
        "runtime_model": {
            "backend": runtime.backend,
            "mode": runtime.mode,
            "cam0_ros_distortion_model": runtime.cam0.ros_distortion_model,
            "cam1_ros_distortion_model": runtime.cam1.ros_distortion_model,
            "metadata": runtime.metadata,
        },
        "stereo_extrinsics": {
            "R_10": rect.P1.__class__.__module__ and np.asarray(R_10).tolist(),  # keep JSON serializable
            "t_10": np.asarray(t_10).reshape(-1).tolist(),
            "baseline_from_translation_norm_m": baseline_from_T,
            "baseline_from_projection_m": baseline_from_P,
            "baseline_relative_error": None if baseline_from_P is None else float(abs(baseline_from_P - baseline_from_T) / max(baseline_from_T, 1e-12)),
        },
        "rectification": {
            "P1": np.asarray(rect.P1).tolist(),
            "P2": np.asarray(rect.P2).tolist(),
            "Q": np.asarray(rect.Q).tolist(),
            "valid_ratio_left": _valid_ratio_from_maps(rect.map1_left, rect.map2_left, raw_cam0.width, raw_cam0.height),
            "valid_ratio_right": _valid_ratio_from_maps(rect.map1_right, rect.map2_right, raw_cam1.width, raw_cam1.height),
            "roi1": rect.roi1,
            "roi2": rect.roi2,
        },
    }


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Convert a stereo Kalibr camchain.yaml into rectified ROS CameraInfo YAML files.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=r"""
EXAMPLES
--------

1) mode=auto (fisheye/equidistant input) — high FOV, balance 0.2:
   Good general-purpose rectification. balance=0.2 crops most of the invalid
   border while retaining a decent field of view. fov-scale=1.0 keeps the
   native rectified focal length.

     %(prog)s --input-camchain camchain.yaml \
              --output-left left.yaml --output-right right.yaml \
              --mode auto --balance 0.2 --fov-scale 1.0

2) mode=auto (fisheye/equidistant input) — maximum FOV retained:
   balance=1.0 keeps the widest possible FOV (more black borders).
   fov-scale < 1.0 zooms in slightly to reduce them.

     %(prog)s --input-camchain camchain.yaml \
              --output-left left.yaml --output-right right.yaml \
              --mode auto --balance 1.0 --fov-scale 0.8

3) mode=auto (radtan/plumb_bob input):
   For standard pinhole cameras, only --alpha matters.
   alpha=0 crops all invalid pixels; alpha=1 keeps the full frame.

     %(prog)s --input-camchain camchain.yaml \
              --output-left left.yaml --output-right right.yaml \
              --mode auto --alpha 0.0

4) mode=surrogate_plumb_bob (fisheye → approximate plumb_bob export):
   Fits a Brown-Conrady (plumb_bob) model to the central 75%% of the fisheye
   mapping. Useful when a downstream node only accepts plumb_bob CameraInfo.
   The fit quality depends on how much of the FOV you include (radius-ratio)
   and the grid density (fit-grid-n). After fitting, the standard backend
   rectification is used, so --alpha controls the crop.

     %(prog)s --input-camchain camchain.yaml \
              --output-left left.yaml --output-right right.yaml \
              --mode surrogate_plumb_bob \
              --surrogate-fit-radius-ratio 0.75 \
              --surrogate-fit-grid-n 31 \
              --alpha 0.5

PARAMETER SUMMARY BY MODE
--------------------------
  mode=auto + equidistant  →  uses --balance, --fov-scale   (ignores --alpha)
  mode=auto + radtan       →  uses --alpha                  (ignores --balance, --fov-scale)
  mode=surrogate_plumb_bob →  uses --alpha, --surrogate-*   (ignores --balance, --fov-scale)
""",
    )

    # ── Required I/O ───────────────────────────────────────────────────────
    parser.add_argument("--input-camchain", required=True, type=str,
                        help="Path to the Kalibr stereo camchain YAML file.")
    parser.add_argument("--output-left", required=True, type=str,
                        help="Output path for the left camera CameraInfo YAML.")
    parser.add_argument("--output-right", required=True, type=str,
                        help="Output path for the right camera CameraInfo YAML.")
    parser.add_argument("--camera-name-left", default="front_camera_left", type=str,
                        help="Value of the 'camera_name' field in the left output YAML. "
                             "(default: %(default)s)")
    parser.add_argument("--camera-name-right", default="front_camera_right", type=str,
                        help="Value of the 'camera_name' field in the right output YAML. "
                             "(default: %(default)s)")

    # ── Mode ───────────────────────────────────────────────────────────────
    parser.add_argument("--mode", choices=["auto", "surrogate_plumb_bob"], default="auto",
                        help="Rectification / export mode (default: %(default)s). "
                             "'auto': use the native Kalibr distortion model directly — "
                             "equidistant inputs use the OpenCV fisheye backend, radtan "
                             "inputs use the standard pinhole backend. "
                             "'surrogate_plumb_bob': fit an approximate Brown-Conrady "
                             "(plumb_bob) model to the central region of a fisheye lens, "
                             "then rectify with the standard pinhole backend. Only valid "
                             "for equidistant/equi inputs. Requires scipy.")

    # ── Surrogate fitting knobs ────────────────────────────────────────────
    parser.add_argument("--surrogate-fit-radius-ratio", default=0.75, type=float,
                        help="[surrogate_plumb_bob only] Ratio (0,1] of the maximum "
                             "inscribed radius used to sample the fisheye mapping for "
                             "the plumb_bob fit. Lower values restrict the fit to a "
                             "smaller central disk → less distortion → better fit, but "
                             "narrower effective FOV. (default: %(default)s)")
    parser.add_argument("--surrogate-fit-grid-n", default=31, type=int,
                        help="[surrogate_plumb_bob only] Side length of the square "
                             "sampling grid (N x N) placed over the fit disk. Higher "
                             "values give a denser sample → more accurate fit at the "
                             "cost of slightly longer computation. (default: %(default)s)")

    # ── Output size ────────────────────────────────────────────────────────
    parser.add_argument("--new-width", default=None, type=int,
                        help="Width of the rectified output image. Must be specified "
                             "together with --new-height. If omitted, the raw calibration "
                             "width is used. (default: same as raw)")
    parser.add_argument("--new-height", default=None, type=int,
                        help="Height of the rectified output image. Must be specified "
                             "together with --new-width. If omitted, the raw calibration "
                             "height is used. (default: same as raw)")

    parser.add_argument("--ros-camera-info-mode", dest="ros_camera_info_mode", action="store_true",
                        help="Enforce that the rectified size equals the raw calibration "
                             "size. This is the standard ROS CameraInfo use-case where K/D "
                             "describe the raw stream and image_proc rectifies it. "
                             "(default: enabled)")
    parser.add_argument("--no-ros-camera-info-mode", dest="ros_camera_info_mode", action="store_false",
                        help="Allow the rectified size to differ from the raw size. Use "
                             "only if you are exporting a non-standard CameraInfo-like "
                             "container (e.g., for a custom remap pipeline).")
    parser.set_defaults(ros_camera_info_mode=True)

    # ── Zero-disparity ─────────────────────────────────────────────────────
    parser.add_argument("--zero-disparity", dest="zero_disparity", action="store_true",
                        help="Force the rectified principal points of both cameras to "
                             "coincide (CALIB_ZERO_DISPARITY). Recommended for most "
                             "stereo matching pipelines. (default: enabled)")
    parser.add_argument("--no-zero-disparity", dest="zero_disparity", action="store_false",
                        help="Do not force coincident principal points. May be needed "
                             "for non-standard stereo geometries.")
    parser.set_defaults(zero_disparity=True)

    # ── Fisheye backend knobs ──────────────────────────────────────────────
    parser.add_argument("--balance", default=0.2, type=float,
                        help="[fisheye backend only] Controls the trade-off between "
                             "retaining the original field-of-view and minimising "
                             "invalid (black) borders after rectification. "
                             "0.0 = crop tightly, only valid pixels remain; "
                             "1.0 = keep the widest possible FOV, more black areas. "
                             "Typical useful range: 0.0–0.5. (default: %(default)s)")
    parser.add_argument("--fov-scale", default=1.0, type=float,
                        help="[fisheye backend only] Divisor applied to the rectified "
                             "focal length. Values < 1.0 zoom in (reduces black borders "
                             "but narrows FOV); values > 1.0 zoom out (wider FOV, more "
                             "border). (default: %(default)s)")

    # ── Standard backend knobs ─────────────────────────────────────────────
    parser.add_argument("--alpha", default=1.0, type=float,
                        help="[standard backend only] Free scaling parameter for "
                             "cv2.stereoRectify. "
                             "0.0 = crop all invalid pixels, the rectified image "
                             "contains only valid data; "
                             "1.0 = retain all source pixels, invalid areas are filled "
                             "with black. Intermediate values blend between the two. "
                             "(default: %(default)s)")

    # ── Optional outputs ───────────────────────────────────────────────────
    parser.add_argument("--save-maps-npz", default=None, type=str,
                        help="If set, save the rectification maps (map1/map2 for each "
                             "camera) plus R1, R2, P1, P2, Q and runtime K/D to a "
                             "compressed .npz file. Useful for offline remap pipelines. "
                             "(default: not saved)")
    parser.add_argument("--diagnostics-json", default=None, type=str,
                        help="If set, write a detailed JSON report with baseline checks, "
                             "valid pixel ratios, and the runtime model metadata. "
                             "(default: not saved)")
    parser.add_argument("--map-type", choices=["16sc2", "32fc1"], default="32fc1",
                        help="Pixel map storage format used by initUndistortRectifyMap. "
                             "'32fc1' stores float maps, easier to inspect and debug; "
                             "'16sc2' stores fixed-point maps, more compact and faster "
                             "for cv2.remap at deployment time. (default: %(default)s)")
    return parser.parse_args()


def main():
    args = _parse_args()
    input_path = Path(args.input_camchain)
    out_left = Path(args.output_left)
    out_right = Path(args.output_right)

    raw_cam0, raw_cam1, R_10, t_10 = _parse_camchain(input_path)

    raw_size = (raw_cam0.width, raw_cam0.height)
    if args.new_width is None and args.new_height is None:
        new_size = raw_size
    elif args.new_width is not None and args.new_height is not None:
        new_size = (int(args.new_width), int(args.new_height))
    else:
        raise ValueError("Provide both --new-width and --new-height together, or neither.")

    if args.ros_camera_info_mode and new_size != raw_size:
        raise ValueError("In --ros-camera-info-mode the rectified size must equal the raw calibration size.")

    runtime = _build_runtime_model(
        raw_cam0, raw_cam1,
        mode=args.mode,
        surrogate_fit_radius_ratio=float(args.surrogate_fit_radius_ratio),
        surrogate_fit_grid_n=int(args.surrogate_fit_grid_n),
    )

    rect = _rectify(
        runtime=runtime,
        R_10=R_10,
        t_10=t_10,
        size=raw_size,
        new_size=new_size,
        zero_disparity=bool(args.zero_disparity),
        balance=float(args.balance),
        fov_scale=float(args.fov_scale),
        alpha=float(args.alpha),
        map_type=args.map_type,
    )

    yml_left = _camera_info_yaml(args.camera_name_left, runtime.cam0, rect.R1, rect.P1)
    yml_right = _camera_info_yaml(args.camera_name_right, runtime.cam1, rect.R2, rect.P2)
    _save_yaml(out_left, yml_left)
    _save_yaml(out_right, yml_right)

    if args.save_maps_npz is not None:
        np.savez_compressed(
            args.save_maps_npz,
            mode=runtime.mode,
            backend=runtime.backend,
            map1_left=rect.map1_left,
            map2_left=rect.map2_left,
            map1_right=rect.map1_right,
            map2_right=rect.map2_right,
            R1=rect.R1, R2=rect.R2, P1=rect.P1, P2=rect.P2, Q=rect.Q,
            K0_runtime=runtime.cam0.K, D0_runtime=runtime.cam0.D,
            K1_runtime=runtime.cam1.K, D1_runtime=runtime.cam1.D,
        )

    diag = _diagnostics(raw_cam0, raw_cam1, runtime, rect, R_10, t_10)
    if args.diagnostics_json is not None:
        with Path(args.diagnostics_json).open("w", encoding="utf-8") as f:
            json.dump(diag, f, indent=2)

    print(json.dumps({
        "status": "ok",
        "mode": runtime.mode,
        "backend": runtime.backend,
        "baseline_from_translation_norm_m": diag["stereo_extrinsics"]["baseline_from_translation_norm_m"],
        "baseline_from_projection_m": diag["stereo_extrinsics"]["baseline_from_projection_m"],
        "valid_ratio_left": diag["rectification"]["valid_ratio_left"],
        "valid_ratio_right": diag["rectification"]["valid_ratio_right"],
    }, indent=2))


if __name__ == "__main__":
    main()
