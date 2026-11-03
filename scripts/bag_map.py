"""Final cartographer map and the robot's path (map frame) from an exploration bag, saved as .npz.

    python3 scripts/bag_map.py <bag dir> <out.npz>
"""
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from bag_animation import compose, latest, read  # noqa: E402

d = read(sys.argv[1])
for k in d:
    d[k].sort(key=lambda s: s[0])
_, grid, res, ox, oy = d["/map"][-1]
trail = []
for t, *o in d["/odom"][::5]:
    m = latest(d["/tf"], t) or (t, 0.0, 0.0, 0.0)
    trail.append(compose(m[1:], o))
np.savez_compressed(sys.argv[2], grid=grid.astype(np.int8), res=res, origin=[ox, oy], trail=np.array(trail))
print(sys.argv[2], grid.shape, res, (ox, oy), len(trail), "poses")
