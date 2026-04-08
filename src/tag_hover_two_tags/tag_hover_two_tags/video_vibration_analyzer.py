#!/usr/bin/env python3
"""
video_vibration_analyzer.py

Offline dual-AprilTag vibration analysis from a recorded video file.

Two AprilTag markers must be simultaneously visible in the video:
  - Reference tag  (stationary, background): defines a local world frame
  - Vibrating tag  (on the structure):        motion to be measured

The relative transform:
    T_vib_ref = inv(T_ref_cam) @ T_vib_cam

cancels all UAV/camera motion, leaving only the true structural vibration
signal — identical math to the live ROS pipeline in tag_hover_two_tags.

Outputs:
  - <stem>_vibration.csv      per-frame relative pose (x, y, z, roll, pitch, yaw)
  - <stem>_displacement.png   displacement in mm over time (x, y, z axes)
  - <stem>_frequency.png      dominant Hz over time + spectrogram

Usage:
    python3 video_vibration_analyzer.py recording.mp4 \\
        --calibration camera.yaml \\
        --tag-size 0.127 \\
        [--ref-id 0] [--vib-id 1] \\
        [--output-dir results/] \\
        [--window-sec 2.0] \\
        [--annotated-video]

Dependencies:
    pip install pupil-apriltags opencv-python numpy scipy matplotlib pyyaml
"""

import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

try:
    from pupil_apriltags import Detector
except ImportError:
    sys.exit(
        "Missing dependency: pip install pupil-apriltags\n"
        "Full install: pip install pupil-apriltags opencv-python numpy scipy matplotlib pyyaml"
    )

try:
    import matplotlib
    matplotlib.use("Agg")  # non-interactive; safe in headless environments
    import matplotlib.pyplot as plt
    from scipy import signal as scipy_signal

    HAS_PLOT = True
except ImportError:
    HAS_PLOT = False
    print("Warning: matplotlib/scipy not installed — plots will be skipped.")
    print("         pip install matplotlib scipy")


# ──────────────────────────────────────────────────────────────────────────────
# Camera calibration loading
# ──────────────────────────────────────────────────────────────────────────────

def load_calibration(yaml_path: str):
    """
    Load camera intrinsic matrix and distortion coefficients from a YAML file.

    Supported formats:
      1. ROS camera_info YAML  (output of `ros2 run camera_calibration cameracalibrator`)
      2. Simple key-value YAML (keys: fx, fy, cx, cy, distortion)

    Returns:
        camera_matrix : (3, 3) float64 ndarray
        dist_coeffs   : (N,)   float64 ndarray
    """
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    # ── ROS camera_info format ────────────────────────────────────────────
    if "camera_matrix" in data:
        K = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
        D = np.array(data["distortion_coefficients"]["data"], dtype=np.float64)

    # ── Simple fx/fy/cx/cy format ─────────────────────────────────────────
    elif "fx" in data:
        fx = float(data["fx"])
        fy = float(data.get("fy", fx))
        cx = float(data["cx"])
        cy = float(data["cy"])
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        D = np.array(data.get("distortion", [0.0, 0.0, 0.0, 0.0, 0.0]), dtype=np.float64)

    else:
        raise ValueError(
            f"Unrecognised calibration format in '{yaml_path}'.\n"
            "Expected ROS camera_info YAML or keys: fx, fy, cx, cy."
        )

    return K, D


# ──────────────────────────────────────────────────────────────────────────────
# Pose estimation
# ──────────────────────────────────────────────────────────────────────────────

def tag_3d_corners(tag_size: float) -> np.ndarray:
    """
    3-D corners of a square AprilTag in its local frame (tag lies in Z=0 plane).
    Corner order matches pupil_apriltags output:
        [0] bottom-left  [1] bottom-right  [2] top-right  [3] top-left
    """
    h = tag_size / 2.0
    return np.array(
        [[-h, -h, 0.0], [h, -h, 0.0], [h, h, 0.0], [-h, h, 0.0]],
        dtype=np.float64,
    )


def estimate_pose(
    detection,
    tag_size: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    max_reproj_error: float = 8.0,
):
    """
    Estimate the 4×4 homogeneous pose T_tag_cam from a single AprilTag detection.

    Uses SOLVEPNP_IPPE_SQUARE (designed for square planar markers, returns the
    two ambiguous solutions) and selects the one with the lower reprojection error.
    Falls back to SOLVEPNP_ITERATIVE if IPPE_SQUARE is unavailable.

    Returns None when the reprojection error exceeds max_reproj_error — this
    rejects estimates from partially occluded, very blurry, or tiny detections.
    """
    obj_pts = tag_3d_corners(tag_size)
    img_pts = detection.corners.astype(np.float64)

    try:
        # SOLVEPNP_ITERATIVE: robust, no corner-ordering constraints, works well
        # for square planar tags viewed roughly face-on (typical inspection use case).
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None
        proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, camera_matrix, dist_coeffs)
        reproj_err = float(np.mean(np.linalg.norm(img_pts - proj.reshape(-1, 2), axis=1)))

    except (cv2.error, AttributeError):
        return None

    if reproj_err > max_reproj_error:
        return None  # likely blur, partial occlusion, or wrong detection

    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T


def relative_transform(T_ref_cam: np.ndarray, T_vib_cam: np.ndarray) -> np.ndarray:
    """
    Compute T_vib_ref = inv(T_ref_cam) @ T_vib_cam.

    Expresses the vibrating tag pose in the reference tag frame.
    This cancels UAV translation, attitude drift, and camera vibration —
    leaving only the true relative motion of the structure.
    """
    return np.linalg.inv(T_ref_cam) @ T_vib_cam


def rotation_to_rpy(R: np.ndarray):
    """Extract roll, pitch, yaw (radians) from a 3×3 rotation matrix."""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll  = np.arctan2( R[2, 1],  R[2, 2])
        pitch = np.arctan2(-R[2, 0],  sy)
        yaw   = np.arctan2( R[1, 0],  R[0, 0])
    else:
        roll  = np.arctan2(-R[1, 2],  R[1, 1])
        pitch = np.arctan2(-R[2, 0],  sy)
        yaw   = 0.0
    return roll, pitch, yaw


# ──────────────────────────────────────────────────────────────────────────────
# Frequency analysis
# ──────────────────────────────────────────────────────────────────────────────

def compute_dominant_frequency(
    times: np.ndarray,
    values: np.ndarray,
    window_sec: float = 2.0,
    overlap: float = 0.5,
    min_freq_hz: float = 0.5,
):
    """
    Sliding-window FFT: estimate the dominant vibration frequency over time.

    Args:
        times       : 1-D array of timestamps in seconds
        values      : 1-D displacement array (same length)
        window_sec  : analysis window width in seconds
        overlap     : fractional overlap between consecutive windows (0–1)
        min_freq_hz : ignore frequencies below this (removes slow drift)

    Returns:
        centers    (N,) — window centre timestamps
        dom_freqs  (N,) — dominant frequency in Hz per window
        amplitudes (N,) — peak FFT magnitude (relative, arbitrary units)
    """
    if len(times) < 8:
        return np.array([]), np.array([]), np.array([])

    dt = float(np.median(np.diff(times)))
    if dt <= 0:
        return np.array([]), np.array([]), np.array([])

    win_n  = max(8, int(window_sec / dt))
    step_n = max(1, int(win_n * (1.0 - overlap)))

    centers, dom_freqs, amplitudes = [], [], []

    for start in range(0, len(values) - win_n + 1, step_n):
        win = values[start : start + win_n] - np.mean(values[start : start + win_n])
        win = win * np.hanning(len(win))  # reduce spectral leakage

        freqs   = np.fft.rfftfreq(len(win), d=dt)
        fft_mag = np.abs(np.fft.rfft(win))

        valid = freqs >= min_freq_hz
        if not np.any(valid):
            continue

        peak_idx   = np.argmax(fft_mag[valid])
        centers.append(times[start + win_n // 2])
        dom_freqs.append(freqs[valid][peak_idx])
        amplitudes.append(fft_mag[valid][peak_idx])

    return np.array(centers), np.array(dom_freqs), np.array(amplitudes)


# ──────────────────────────────────────────────────────────────────────────────
# Annotation helper
# ──────────────────────────────────────────────────────────────────────────────

def draw_detections(frame: np.ndarray, detections, ref_id: int, vib_id: int):
    """Overlay detection outlines and labels on a video frame (returns copy)."""
    out = frame.copy()
    for det in detections:
        corners = det.corners.astype(int)
        tid     = det.tag_id
        is_ref  = tid == ref_id
        color   = (0, 220, 0) if is_ref else (0, 165, 255)   # green / orange
        label   = f"REF id={tid}" if is_ref else f"VIB id={tid}"

        cv2.polylines(out, [corners.reshape(-1, 1, 2)], True, color, 2)
        cx, cy = det.center.astype(int)
        cv2.putText(out, label, (cx - 40, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        for px, py in corners:
            cv2.circle(out, (int(px), int(py)), 4, color, -1)
    return out


# ──────────────────────────────────────────────────────────────────────────────
# Output: CSV
# ──────────────────────────────────────────────────────────────────────────────

CSV_HEADER = [
    "frame", "time_sec",
    "x", "y", "z",
    "roll", "pitch", "yaw",
    "ref_detected", "vib_detected",
]


def save_csv(rows: list, output_path: Path):
    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_HEADER)
        writer.writeheader()
        writer.writerows(rows)
    print(f"  CSV            → {output_path}")


# ──────────────────────────────────────────────────────────────────────────────
# Output: plots
# ──────────────────────────────────────────────────────────────────────────────

def plot_displacement(times: np.ndarray, poses: np.ndarray,
                      output_path: Path):
    """
    Figure 1 — Displacement over time.
    Three stacked subplots: X, Y, Z relative displacement in millimetres.
    """
    fig, axes = plt.subplots(3, 1, figsize=(13, 7), sharex=True)
    fig.suptitle(
        "Relative Displacement — Vibrating Tag in Reference Frame",
        fontsize=13, fontweight="bold",
    )

    axis_cfg = [
        ("X  (lateral)",  poses[:, 0], "steelblue"),
        ("Y  (vertical)", poses[:, 1], "tomato"),
        ("Z  (depth)",    poses[:, 2], "seagreen"),
    ]
    for ax, (label, data, color) in zip(axes, axis_cfg):
        ax.plot(times, data * 1000.0, color=color, linewidth=0.8, alpha=0.9)
        ax.set_ylabel(f"{label}\n(mm)", fontsize=9)
        ax.axhline(0, color="black", linewidth=0.5, linestyle="--", alpha=0.5)
        ax.grid(True, alpha=0.3)
        rms_mm = np.std(data) * 1000.0
        ax.text(0.99, 0.92, f"RMS = {rms_mm:.2f} mm",
                transform=ax.transAxes, ha="right", fontsize=8, color=color)

    axes[-1].set_xlabel("Time (s)", fontsize=10)
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    print(f"  Displacement   → {output_path}")


def plot_frequency(times: np.ndarray, poses: np.ndarray,
                   window_sec: float, output_path: Path):
    """
    Figure 2 — Frequency analysis.
      Top    : Dominant vibration frequency over time (Hz vs time).
               Point size and colour encode amplitude (brighter = stronger).
      Bottom : Spectrogram of the axis with the highest RMS displacement.
    """
    # Choose the most active translation axis automatically
    rms = [np.std(poses[:, i]) for i in range(3)]
    primary_idx  = int(np.argmax(rms))
    axis_labels  = ["X (lateral)", "Y (vertical)", "Z (depth)"]
    primary_sig  = poses[:, primary_idx]
    primary_name = axis_labels[primary_idx]

    centers, dom_freqs, amplitudes = compute_dominant_frequency(
        times, primary_sig, window_sec=window_sec
    )

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 8))
    fig.suptitle(
        f"Vibration Frequency Analysis   (primary axis: {primary_name})",
        fontsize=13, fontweight="bold",
    )

    # ── top: dominant frequency over time ────────────────────────────────
    if len(centers) > 0:
        amp_norm = amplitudes / (amplitudes.max() + 1e-12)
        sc = ax1.scatter(
            centers, dom_freqs,
            c=amp_norm, cmap="plasma",
            s=15 + 55 * amp_norm, alpha=0.85, edgecolors="none",
        )
        cbar = plt.colorbar(sc, ax=ax1, pad=0.01)
        cbar.set_label("Relative amplitude", fontsize=8)
    else:
        ax1.text(0.5, 0.5, "Not enough data for frequency analysis",
                 ha="center", va="center", transform=ax1.transAxes, fontsize=11)

    ax1.set_ylabel("Dominant frequency (Hz)", fontsize=10)
    ax1.set_xlabel("Time (s)", fontsize=10)
    ax1.set_ylim(bottom=0)
    ax1.grid(True, alpha=0.3)

    # ── bottom: spectrogram ───────────────────────────────────────────────
    if len(times) > 16:
        dt      = float(np.median(np.diff(times)))
        fs      = 1.0 / dt
        nperseg = min(256, max(8, len(primary_sig) // 6))
        f_s, t_s, Sxx = scipy_signal.spectrogram(
            primary_sig - np.mean(primary_sig),
            fs=fs, nperseg=nperseg, noverlap=nperseg // 2, scaling="density",
        )
        t_s += times[0]  # shift to recording timestamps
        power_db = 10.0 * np.log10(np.maximum(Sxx, 1e-20))
        pcm = ax2.pcolormesh(t_s, f_s, power_db,
                             shading="gouraud", cmap="inferno")
        cbar2 = plt.colorbar(pcm, ax=ax2, pad=0.01)
        cbar2.set_label("Power (dB)", fontsize=8)
        ax2.set_ylim(0, min(fs / 2.0, 50.0))  # cap display at 50 Hz
        ax2.set_ylabel("Frequency (Hz)", fontsize=10)
        ax2.set_xlabel("Time (s)", fontsize=10)
        ax2.set_title("Spectrogram", fontsize=10)
    else:
        ax2.text(0.5, 0.5, "Not enough frames for spectrogram",
                 ha="center", va="center", transform=ax2.transAxes, fontsize=11)

    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    print(f"  Frequency plot → {output_path}")


# ──────────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Offline dual-AprilTag vibration analysis from video.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("video",
                   help="Input video file (MP4, AVI, MOV, …)")
    p.add_argument("--calibration", required=True,
                   help="Camera calibration YAML (ROS camera_info or fx/fy/cx/cy format)")
    p.add_argument("--tag-size", type=float, required=True,
                   help="Physical side length of the AprilTag in metres "
                        "(both tags assumed the same size)")
    p.add_argument("--ref-id", type=int, default=0,
                   help="AprilTag ID of the stationary reference tag")
    p.add_argument("--vib-id", type=int, default=1,
                   help="AprilTag ID of the vibrating structure tag")
    p.add_argument("--tag-family", default="tag36h11",
                   help="AprilTag family string")
    p.add_argument("--output-dir", default=None,
                   help="Directory for all outputs (default: same folder as video)")
    p.add_argument("--window-sec", type=float, default=2.0,
                   help="Sliding-window width for frequency analysis in seconds")
    p.add_argument("--max-reproj-error", type=float, default=8.0,
                   help="Reject pose estimates with reprojection error above this "
                        "(pixels). Increase to 12-15 for very blurry/small tags.")
    p.add_argument("--quad-decimate", type=float, default=1.0,
                   help="AprilTag quad decimation factor. "
                        "1.0 = full resolution (best for small tags, slower). "
                        "2.0 = half resolution (faster but misses small tags).")
    p.add_argument("--annotated-video", action="store_true",
                   help="Save a copy of the video with tag detections drawn on it")
    p.add_argument("--no-plots", action="store_true",
                   help="Skip plot generation (CSV output still saved)")
    return p.parse_args()


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()

    video_path = Path(args.video)
    if not video_path.exists():
        sys.exit(f"Error: video not found: {video_path}")

    output_dir = Path(args.output_dir) if args.output_dir else video_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = video_path.stem

    print(f"\n{'='*62}")
    print(f"  Video        : {video_path.name}")
    print(f"  Calibration  : {args.calibration}")
    print(f"  Tag size     : {args.tag_size} m")
    print(f"  Tag family   : {args.tag_family}")
    print(f"  Ref ID={args.ref_id}  |  Vib ID={args.vib_id}")
    print(f"  Output dir   : {output_dir}")
    print(f"{'='*62}\n")

    # ── Load calibration ─────────────────────────────────────────────────
    camera_matrix, dist_coeffs = load_calibration(args.calibration)
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    print(f"Camera: fx={fx:.1f}  fy={fy:.1f}  cx={cx:.1f}  cy={cy:.1f}")
    print(f"Dist  : {dist_coeffs.tolist()}\n")

    # ── Set up AprilTag detector ──────────────────────────────────────────
    detector_kwargs = dict(
        families=args.tag_family,
        nthreads=4,
        quad_decimate=args.quad_decimate,
        quad_sigma=0.5,      # mild blur improves detection of slightly fuzzy tags
        refine_edges=True,
    )
    try:
        detector = Detector(**detector_kwargs, decode_sharpening=0.25)
    except TypeError:
        # Older pupil-apriltags versions don't have decode_sharpening
        detector = Detector(**detector_kwargs)

    # ── Open video ───────────────────────────────────────────────────────
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        sys.exit(f"Error: cannot open video: {video_path}")

    fps          = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width        = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height       = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Video : {width}x{height} @ {fps:.1f} fps  ({total_frames} frames  "
          f"~{total_frames/fps:.1f}s)\n")

    # ── Optional annotated video writer ──────────────────────────────────
    writer = None
    if args.annotated_video:
        ann_path = output_dir / f"{stem}_annotated.mp4"
        fourcc   = cv2.VideoWriter_fourcc(*"mp4v")
        writer   = cv2.VideoWriter(str(ann_path), fourcc, fps, (width, height))

    # ── Per-frame processing ──────────────────────────────────────────────
    csv_rows            = []
    good_times          = []
    good_poses          = []   # list of [x, y, z, roll, pitch, yaw]

    n_ref = n_vib = n_both = n_pose_ok = 0

    for frame_idx in range(total_frames):
        ret, frame = cap.read()
        if not ret:
            break

        time_sec = frame_idx / fps
        gray     = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = detector.detect(gray)

        # Index by tag ID; keep detection with lowest Hamming distance if duplicates
        tag_map = {}
        for det in detections:
            tid = det.tag_id
            if tid not in tag_map or det.hamming < tag_map[tid].hamming:
                tag_map[tid] = det

        ref_ok = args.ref_id in tag_map
        vib_ok = args.vib_id in tag_map
        n_ref  += ref_ok
        n_vib  += vib_ok

        row = {
            "frame":        frame_idx,
            "time_sec":     f"{time_sec:.4f}",
            "x": "", "y": "", "z": "",
            "roll": "", "pitch": "", "yaw": "",
            "ref_detected": int(ref_ok),
            "vib_detected": int(vib_ok),
        }

        if ref_ok and vib_ok:
            n_both += 1
            T_ref = estimate_pose(tag_map[args.ref_id], args.tag_size,
                                  camera_matrix, dist_coeffs,
                                  args.max_reproj_error)
            T_vib = estimate_pose(tag_map[args.vib_id], args.tag_size,
                                  camera_matrix, dist_coeffs,
                                  args.max_reproj_error)

            if T_ref is not None and T_vib is not None:
                n_pose_ok += 1
                T_rel          = relative_transform(T_ref, T_vib)
                x, y, z        = T_rel[:3, 3]
                roll, pitch, yaw = rotation_to_rpy(T_rel[:3, :3])

                row.update({
                    "x":     f"{x:.6f}",
                    "y":     f"{y:.6f}",
                    "z":     f"{z:.6f}",
                    "roll":  f"{roll:.6f}",
                    "pitch": f"{pitch:.6f}",
                    "yaw":   f"{yaw:.6f}",
                })
                good_times.append(time_sec)
                good_poses.append([x, y, z, roll, pitch, yaw])

        csv_rows.append(row)

        # Annotated video frame
        if writer is not None:
            ann = draw_detections(frame, detections, args.ref_id, args.vib_id)
            pose_txt = (f"t={time_sec:.2f}s  "
                        f"ref={'OK' if ref_ok else '--'}  "
                        f"vib={'OK' if vib_ok else '--'}  "
                        f"pose={'OK' if (ref_ok and vib_ok and row['x']) else '--'}")
            cv2.putText(ann, pose_txt, (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            writer.write(ann)

        if frame_idx % 150 == 0:
            pct = 100 * frame_idx / max(total_frames, 1)
            print(f"  {frame_idx:>5}/{total_frames}  ({pct:.0f}%)  "
                  f"valid poses: {n_pose_ok}", end="\r", flush=True)

    cap.release()
    if writer is not None:
        writer.release()
        print(f"\n  Annotated video → {ann_path}")

    # ── Summary ───────────────────────────────────────────────────────────
    processed = len(csv_rows)
    print(f"\n\n{'='*62}")
    print(f"  Frames processed    : {processed}")
    print(f"  Ref tag detected    : {n_ref:>5}  ({100*n_ref/max(processed,1):.1f}%)")
    print(f"  Vib tag detected    : {n_vib:>5}  ({100*n_vib/max(processed,1):.1f}%)")
    print(f"  Both detected       : {n_both:>5}  ({100*n_both/max(processed,1):.1f}%)")
    print(f"  Valid pose pairs    : {n_pose_ok:>5}  ({100*n_pose_ok/max(processed,1):.1f}%)")
    print(f"{'='*62}\n")

    if n_pose_ok == 0:
        print("WARNING: No valid pose pairs produced. Common causes:")
        print("  • Tag IDs wrong        → check --ref-id and --vib-id")
        print("  • Tag family mismatch  → check --tag-family (default: tag36h11)")
        print("  • Tags too small       → try --quad-decimate 1.0 (already default)")
        print("  • Very blurry tags     → try --max-reproj-error 15")
        print("  • Wrong calibration    → verify camera.yaml matches this camera")
        print("  • Both tags not in frame simultaneously → check video content\n")

    # ── Save CSV ──────────────────────────────────────────────────────────
    csv_path = output_dir / f"{stem}_vibration.csv"
    save_csv(csv_rows, csv_path)

    # ── Generate plots ────────────────────────────────────────────────────
    if not args.no_plots and n_pose_ok >= 4:
        poses_arr = np.array(good_poses)
        times_arr = np.array(good_times)

        plot_displacement(
            times_arr, poses_arr,
            output_path=output_dir / f"{stem}_displacement.png",
        )
        if HAS_PLOT:
            plot_frequency(
                times_arr, poses_arr,
                window_sec=args.window_sec,
                output_path=output_dir / f"{stem}_frequency.png",
            )
    elif n_pose_ok < 4:
        print("  Not enough valid frames for plots.")

    print("\nDone.\n")


if __name__ == "__main__":
    main()
