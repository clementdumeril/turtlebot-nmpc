# Extra crossing runs, 31 October 2026

The 30 October benchmark had only two NMPC runs per case, both without contact at 0.5 m/s. After adding the real-robot safety code (stale-sensor watchdog, zero command on exit, clock-offset handling) I re-ran the 0.5 m/s crossing to check nothing had changed. Same world, same scenario, `bash scripts/run_benchmark.sh` with `SCENARIOS=crossing`.

| Folder | Code | Runs | Contact | Closest per run [cm] |
|---|---|---|---|---|
| [nmpc_final](nmpc_final/summary.md) | current (`clock_offsets`, minimum of receive time - stamp) | 4 | 1 | +32.1, -2.2, +12.2, +3.6 |
| [nmpc_clock_bug](nmpc_clock_bug/summary.md) | intermediate version, see below | 5 | 3 | -2.2, -7.3, +4.6, -1.0, +10.6 |
| [mppi](mppi/summary.md) | Nav2 MPPI, unchanged | 1 | 1 | -19.8 |

The intermediate version timed the predictions from the last scan stamp plus the time since it was received, which assumes the scan arrives with no delay, so the predicted person probably lagged behind the real one; contact went up to 3 of 5. The current version estimates the clock offset as the minimum of (receive time - stamp) over the last 10 s, which keeps the transport delay in; with the simulation's synced clocks it times the prediction as the 30 October code did.

All NMPC runs with prediction comparable to the 30 October code (2 benchmark runs, 1 recorded run, the 4 in `nmpc_final`): contact in 1 of 7, closest -2.2 cm in that run, median +12.2 cm. The person does not yield, so slowing down too close to their line is enough for contact. Nav2 MPPI: contact in 3 of 3 runs.
