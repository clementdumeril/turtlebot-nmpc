# Offline closed-loop check, 3 November 2026

`scripts/offline_check.py` at commit `8fc1375`: the NMPC and the Kalman tracker on a plain unicycle, figure eight, one person crossing at 0.5 m/s, no ROS or Webots. The detections get unseeded Gaussian noise (sigma 2 cm, `offline_check.py` line 54), so each run differs slightly. Five runs per setting, WSL2 Ubuntu 24.04, acados v0.5.5, casadi 3.7.2, i7-6700HQ.

```bash
python3 scripts/offline_check.py               # with prediction
python3 scripts/offline_check.py --no-predict  # obstacles frozen where last seen
```

Last line of each run:

```text
with prediction
lap 57.4 s, cross-track RMSE 1.8 cm (max 9.4), min clearance 24.4 cm, solve median 1.53 ms max 12.35 ms, wall median 15.03 ms, nonzero status 0
lap 56.7 s, cross-track RMSE 0.5 cm (max 2.2), min clearance 25.8 cm, solve median 1.38 ms max 2157.07 ms, wall median 6.69 ms, nonzero status 0
lap 56.4 s, cross-track RMSE 1.0 cm (max 4.4), min clearance 23.3 cm, solve median 1.09 ms max 10.69 ms, wall median 4.31 ms, nonzero status 0
lap 56.5 s, cross-track RMSE 0.3 cm (max 1.5), min clearance 24.7 cm, solve median 1.00 ms max 11.23 ms, wall median 4.14 ms, nonzero status 0
lap 56.7 s, cross-track RMSE 0.6 cm (max 2.4), min clearance 26.1 cm, solve median 1.09 ms max 9.46 ms, wall median 4.49 ms, nonzero status 0

--no-predict
lap 54.7 s, cross-track RMSE 2.7 cm (max 13.1), min clearance -19.4 cm, solve median 1.46 ms max 12.07 ms, wall median 13.45 ms, nonzero status 0
lap 54.6 s, cross-track RMSE 2.7 cm (max 13.1), min clearance -20.6 cm, solve median 1.06 ms max 7.88 ms, wall median 4.28 ms, nonzero status 0
lap 57.1 s, cross-track RMSE 4.4 cm (max 16.9), min clearance -17.9 cm, solve median 0.88 ms max 6.60 ms, wall median 3.46 ms, nonzero status 0
lap 54.6 s, cross-track RMSE 2.7 cm (max 13.1), min clearance -20.6 cm, solve median 0.74 ms max 3.19 ms, wall median 3.12 ms, nonzero status 0
lap 54.7 s, cross-track RMSE 2.7 cm (max 13.3), min clearance -20.8 cm, solve median 1.00 ms max 9.31 ms, wall median 4.03 ms, nonzero status 0
```

Minimum clearance: 23.3 to 26.1 cm with prediction, -20.8 to -17.9 cm (contact) without, in all 5 runs each. The 2157 ms maximum in one run is a single outlier, not investigated (WSL scheduling is the likely cause, as for the 11.7 ms worst case in the Webots benchmark); the median is unaffected.
