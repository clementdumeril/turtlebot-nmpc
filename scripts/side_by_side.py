"""Phone video of a real run next to the logged odometry path, as an animated WebP.

    PYTHONPATH=src/tb_nmpc python3 scripts/side_by_side.py <video.MOV> <run dir> <out.webp> \
        --start "2026-11-01T16:08:52-08:00" --crop 1080:900:0:600 --shape circle --size 0.5

--start is the phone's capture time (QuickTime creationdate, 1 s resolution), which places the
video on the run's clock. Needs ffmpeg (imageio-ffmpeg's binary is used if ffmpeg is not on PATH).
"""
from __future__ import annotations

import argparse
import datetime as dt
import shutil
import subprocess
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from metrics import load  # noqa: E402

from tb_nmpc.reference import anchored  # noqa: E402


def ffmpeg() -> str:
    exe = shutil.which("ffmpeg")
    if exe:
        return exe
    import imageio_ffmpeg
    return imageio_ffmpeg.get_ffmpeg_exe()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video"); ap.add_argument("run"); ap.add_argument("out")
    ap.add_argument("--start", required=True, help="video capture time, ISO 8601 with offset")
    ap.add_argument("--crop", default="1080:900:0:600", help="ffmpeg crop w:h:x:y on the rotated video")
    ap.add_argument("--shape", default="circle"); ap.add_argument("--size", type=float, default=0.5)
    ap.add_argument("--speed", type=float, default=2.5); ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--height", type=int, default=450)
    a = ap.parse_args()

    d, meta = load(Path(a.run))
    t = d["t"]
    x0, y0, th0 = d["odom_x"][0], d["odom_y"][0], d["odom_yaw"][0]
    c, s = np.cos(-th0), np.sin(-th0)
    X = c * (d["odom_x"] - x0) - s * (d["odom_y"] - y0)
    Y = s * (d["odom_x"] - x0) + c * (d["odom_y"] - y0)
    TH = d["odom_yaw"] - th0
    offset = dt.datetime.fromisoformat(a.start).timestamp() - t[0]   # video t = 0 on the run clock
    ref = anchored(a.size, 1.0, (0, 0, 0), a.shape).xy

    cw, ch = (int(v) for v in a.crop.split(":")[:2])
    H = a.height
    W = int(round(cw * H / ch)) // 2 * 2
    cmd = [ffmpeg(), "-loglevel", "error", "-i", a.video, "-vf",
           f"setpts=PTS/{a.speed},fps={a.fps},crop={a.crop},scale={W}:{H}", "-f", "rawvideo", "-pix_fmt", "rgb24", "-"]
    raw = subprocess.run(cmd, capture_output=True, check=True).stdout
    frames = np.frombuffer(raw, np.uint8).reshape(-1, H, W, 3)

    fig, ax = plt.subplots(figsize=(H / 100, H / 100), dpi=100)
    fig.subplots_adjust(0.13, 0.1, 0.97, 0.9)
    pad = 0.12
    out = []
    for i, f in enumerate(frames):
        tr = t[0] + offset + i * a.speed / a.fps
        n = int(np.searchsorted(t, tr))
        ax.clear()
        ax.plot(ref[:, 0], ref[:, 1], color="0.6", ls="--", lw=1)
        if n > 0:
            ax.plot(X[:n], Y[:n], color="#1f5fbf", lw=2)
            k = min(n, len(t)) - 1
            ax.plot(X[k], Y[k], marker=(3, 0, np.degrees(TH[k]) - 90), ms=11, color="#d97b29")
        ax.set_xlim(ref[:, 0].min() - pad, ref[:, 0].max() + pad)
        ax.set_ylim(ref[:, 1].min() - pad, ref[:, 1].max() + pad)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.tick_params(labelsize=7)
        ax.set_title(f"logged odometry   t = {max(0.0, tr - t[0]):4.1f} s", fontsize=9)
        fig.canvas.draw()
        plot = np.asarray(fig.canvas.buffer_rgba())[..., :3]
        out.append(np.hstack([f, plot]))
    frames_out = np.stack(out)
    enc = [ffmpeg(), "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
           "-s", f"{frames_out.shape[2]}x{frames_out.shape[1]}", "-r", str(a.fps), "-i", "-",
           "-c:v", "libwebp_anim", "-quality", "60", "-compression_level", "6", "-loop", "0", a.out]
    subprocess.run(enc, input=frames_out.tobytes(), check=True)
    print(a.out, len(frames_out), "frames, video starts", f"{offset:+.1f} s after the log")


if __name__ == "__main__":
    main()
