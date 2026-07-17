#!/usr/bin/env python3
"""
csv_vibration_analyzer.py

Path A post-processing: turn the CSV logged by the live measurement node
(`ros2 run tag_hover_two_tags relative_vibration_pose`, saved in
~/.ros/tag_hover_two_tags/) into the same results the video analyzer produces:

  - <stem>_displacement.png   relative displacement in mm over time (x, y, z)
  - <stem>_frequency.png      dominant vibration frequency over time + spectrogram
  - printed summary           duration, sample rate, RMS and dominant Hz per axis

The CSV already contains the relative pose of the vibrating tag (ID 1) in the
reference tag frame (ID 0) — this script only does the signal analysis, with
the same math as tools/video_vibration_analyzer.py.

Usage:
    python3 csv_vibration_analyzer.py ~/.ros/tag_hover_two_tags/relative_vibration_XXXXXXXX_XXXXXX.csv \
        [--output-dir results/] [--window-sec 2.0]

Dependencies:
    pip install numpy scipy matplotlib
"""

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")  # non-interactive; safe in headless environments
    import matplotlib.pyplot as plt
    from scipy import signal as scipy_signal
except ImportError:
    sys.exit("Missing dependency: pip install numpy scipy matplotlib")


# ──────────────────────────────────────────────────────────────────────────────
# Frequency analysis (same as video_vibration_analyzer.py)
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

        peak_idx = np.argmax(fft_mag[valid])
        centers.append(times[start + win_n // 2])
        dom_freqs.append(freqs[valid][peak_idx])
        amplitudes.append(fft_mag[valid][peak_idx])

    return np.array(centers), np.array(dom_freqs), np.array(amplitudes)


def overall_dominant_frequency(times: np.ndarray, values: np.ndarray,
                               min_freq_hz: float = 0.5) -> float:
    """Single FFT over the whole recording; returns the dominant Hz (or nan)."""
    if len(times) < 8:
        return float("nan")
    dt  = float(np.median(np.diff(times)))
    win = (values - np.mean(values)) * np.hanning(len(values))
    freqs   = np.fft.rfftfreq(len(win), d=dt)
    fft_mag = np.abs(np.fft.rfft(win))
    valid   = freqs >= min_freq_hz
    if not np.any(valid):
        return float("nan")
    return float(freqs[valid][np.argmax(fft_mag[valid])])


# ──────────────────────────────────────────────────────────────────────────────
# Plots (same layout as video_vibration_analyzer.py)
# ──────────────────────────────────────────────────────────────────────────────

def plot_displacement(times: np.ndarray, poses: np.ndarray, output_path: Path):
    """Three stacked subplots: X, Y, Z relative displacement in millimetres."""
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
        ax.axhline(np.mean(data) * 1000.0, color="black",
                   linewidth=0.5, linestyle="--", alpha=0.5)
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
    Top    : Dominant vibration frequency over time (Hz vs time).
    Bottom : Spectrogram of the axis with the highest RMS displacement.
    """
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

    if len(times) > 16:
        dt      = float(np.median(np.diff(times)))
        fs      = 1.0 / dt
        nperseg = min(256, max(8, len(primary_sig) // 6))
        f_s, t_s, Sxx = scipy_signal.spectrogram(
            primary_sig - np.mean(primary_sig),
            fs=fs, nperseg=nperseg, noverlap=nperseg // 2, scaling="density",
        )
        t_s += times[0]
        power_db = 10.0 * np.log10(np.maximum(Sxx, 1e-20))
        pcm = ax2.pcolormesh(t_s, f_s, power_db,
                             shading="gouraud", cmap="inferno")
        cbar2 = plt.colorbar(pcm, ax=ax2, pad=0.01)
        cbar2.set_label("Power (dB)", fontsize=8)
        ax2.set_ylim(0, min(fs / 2.0, 50.0))
        ax2.set_ylabel("Frequency (Hz)", fontsize=10)
        ax2.set_xlabel("Time (s)", fontsize=10)
        ax2.set_title("Spectrogram", fontsize=10)
    else:
        ax2.text(0.5, 0.5, "Not enough samples for spectrogram",
                 ha="center", va="center", transform=ax2.transAxes, fontsize=11)

    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    print(f"  Frequency plot → {output_path}")


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Vibration analysis of a relative_vibration_pose CSV log.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("csv",
                   help="CSV from relative_vibration_pose "
                        "(~/.ros/tag_hover_two_tags/relative_vibration_*.csv)")
    p.add_argument("--output-dir", default=None,
                   help="Directory for the plots (default: same folder as the CSV)")
    p.add_argument("--window-sec", type=float, default=2.0,
                   help="Sliding-window width for frequency analysis in seconds")
    return p.parse_args()


def main():
    args = parse_args()

    csv_path = Path(args.csv).expanduser()
    if not csv_path.exists():
        sys.exit(f"Error: CSV not found: {csv_path}")
    if csv_path.stat().st_size == 0:
        sys.exit(f"Error: {csv_path.name} is empty (0 bytes) — the run produced no "
                 "poses. See BENCH_TEST_OAK.md 'Si algo falla'.")

    data = np.genfromtxt(csv_path, delimiter=",", names=True)
    if data.size == 0:
        sys.exit(f"Error: {csv_path.name} has a header but no data rows.")
    data = np.atleast_1d(data)

    required = ("stamp_sec", "x", "y", "z", "roll", "pitch", "yaw")
    missing = [c for c in required if c not in (data.dtype.names or ())]
    if missing:
        sys.exit(f"Error: CSV is missing columns {missing} — is this a "
                 "relative_vibration_pose log?")

    times = data["stamp_sec"] - data["stamp_sec"][0]
    poses = np.column_stack([data[c] for c in ("x", "y", "z", "roll", "pitch", "yaw")])

    ok = np.isfinite(times) & np.all(np.isfinite(poses), axis=1)
    times, poses = times[ok], poses[ok]
    order = np.argsort(times)
    times, poses = times[order], poses[order]

    if len(times) < 2:
        sys.exit("Error: fewer than 2 valid samples — nothing to analyse.")

    output_dir = Path(args.output_dir).expanduser() if args.output_dir else csv_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = csv_path.stem

    dt   = float(np.median(np.diff(times)))
    rate = 1.0 / dt if dt > 0 else float("nan")

    print(f"\n{'='*62}")
    print(f"  CSV        : {csv_path}")
    print(f"  Samples    : {len(times)}")
    print(f"  Duration   : {times[-1]:.1f} s")
    print(f"  Rate       : {rate:.1f} Hz  (Nyquist limit: {rate/2:.1f} Hz)")
    print(f"{'='*62}\n")

    axis_names = ["X (lateral)", "Y (vertical)", "Z (depth)"]
    for i, name in enumerate(axis_names):
        rms_mm = np.std(poses[:, i]) * 1000.0
        dom_hz = overall_dominant_frequency(times, poses[:, i])
        print(f"  {name:13s}  RMS = {rms_mm:6.2f} mm   dominant ≈ {dom_hz:5.2f} Hz")
    print()

    plot_displacement(times, poses, output_dir / f"{stem}_displacement.png")
    plot_frequency(times, poses, args.window_sec,
                   output_dir / f"{stem}_frequency.png")

    print("\nDone.\n")


if __name__ == "__main__":
    main()
