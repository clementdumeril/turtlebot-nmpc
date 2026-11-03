"""Write a COLMAP text model with the robot's camera poses (cartographer base_link x Lab 6 mount)."""
import csv, json, sqlite3, sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

LAB6, RUN, WS = Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3])
level = Rotation.from_euler("xyz", [-np.pi / 2, 0.0, -np.pi / 2]).as_matrix()
tilt = Rotation.from_euler("xyz", [8.12, 0.0, -0.17], degrees=True).as_matrix()
T_BASE_CAM = np.eye(4); T_BASE_CAM[:3, :3] = level @ tilt; T_BASE_CAM[:3, 3] = [0.115, 0, 0]
cal = json.load(open(LAB6 / "reconstruction" / "calibration" / "calibration.json"))
K, d = np.array(cal["K"]), cal["dist"]
db = sqlite3.connect(WS / "database.db")
ids = {n: (i, c) for i, n, c in db.execute("select image_id, name, camera_id from images")}
cams = {c for _, c in ids.values()}
assert len(cams) == 1, cams
out = WS / "sparse_known"; out.mkdir(exist_ok=True)
params = [K[0, 0], K[1, 1], K[0, 2], K[1, 2], d[0], d[1], d[2], d[3], d[4], 0, 0, 0]
(out / "cameras.txt").write_text(f"{cams.pop()} FULL_OPENCV 640 480 " + " ".join(f"{p:.10g}" for p in params) + "\n")
lines = []
for row in csv.DictReader(open(LAB6 / "captures" / RUN / "poses.csv")):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([float(row[k]) for k in ("qx", "qy", "qz", "qw")]).as_matrix()
    T[:3, 3] = [float(row[k]) for k in ("x", "y", "z")]
    Tcw = np.linalg.inv(T @ T_BASE_CAM)
    qx, qy, qz, qw = Rotation.from_matrix(Tcw[:3, :3]).as_quat()
    iid, cid = ids[row["file"]]
    lines += [f"{iid} {qw} {qx} {qy} {qz} {Tcw[0,3]} {Tcw[1,3]} {Tcw[2,3]} {cid} {row['file']}", ""]
(out / "images.txt").write_text("\n".join(lines) + "\n")
(out / "points3D.txt").write_text("")
print(len(lines) // 2, "images written")
