# Optimal control problem

The full formulation behind [`ocp.py`](../src/tb_nmpc/tb_nmpc/ocp.py) and [`tracker.py`](../src/tb_nmpc/tb_nmpc/tracker.py). Back to the [README](../README.md).

Every 0.1 s, from the measured state $`\hat{x}(t)`$, acados solves

```math
\begin{aligned}
\min_{x,\,u,\,s}\quad & \sum_{k=0}^{N-1} \big\lVert y(x_k,u_k) - y^{\mathrm{ref}}_k \big\rVert^2_{W}
\;+\; \big\lVert y_N(x_N) - y^{\mathrm{ref}}_N \big\rVert^2_{W_N}
\;+\; \sum_{k=0}^{N}\sum_{j=1}^{M} \left(\rho_1\, s_{k,j} + \rho_2\, s_{k,j}^2\right) \\[6pt]
\text{s.t.}\quad & x_0 = \hat{x}(t), \qquad x_{k+1} = \Phi_{\mathrm{RK4}}(x_k, u_k, \Delta t) \\[2pt]
& \big\lVert p_k - \hat{o}_j(\tau_{k,j}) \big\rVert \;\ge\; \hat{r}_j(\tau_{k,j}) + r_{\mathrm{robot}} + 0.05\ \mathrm{m} - s_{k,j}, \qquad s_{k,j} \ge 0 \\[2pt]
& 0 \le v_k \le v_{\max}, \quad |\omega_k| \le \omega_{\max}, \quad |a_k| \le a_{\max}, \quad |\alpha_k| \le \alpha_{\max}
\end{aligned}
```

on the unicycle with the velocities as states, so the acceleration limits are hard and the command is smooth:

```math
x = \begin{bmatrix} p_x & p_y & \theta & v & \omega \end{bmatrix}^{\top}, \qquad
u = \begin{bmatrix} a & \alpha \end{bmatrix}^{\top}, \qquad
\dot{p}_x = v\cos\theta,\;\; \dot{p}_y = v\sin\theta,\;\; \dot{\theta} = \omega,\;\; \dot{v} = a,\;\; \dot{\omega} = \alpha
```

```math
y(x, u) = \begin{bmatrix} p_x & p_y & \cos\theta & \sin\theta & v & \omega & a & \alpha \end{bmatrix}^{\top}, \qquad
y_N(x) = \begin{bmatrix} p_x & p_y & \cos\theta & \sin\theta & v \end{bmatrix}^{\top}
```

The heading is tracked through $`\cos\theta, \sin\theta`$, so there is no angle wrapping. Each obstacle $`j`$ (position $`o_j`$, velocity $`\hat{v}_j`$, velocity uncertainty $`\sigma_{v,j}`$, last update $`t_j`$) is predicted to the time of stage $`k`$:

```math
\tau_{k,j} = t - t_j + k\,\Delta t, \qquad
\hat{o}_j(\tau) = o_j + \hat{v}_j\,\tau, \qquad
\hat{r}_j(\tau) = r_j + \min\left(0.25\ \mathrm{m},\ \sigma_{v,j}\,\tau\right)
```

The reference $`y^{\mathrm{ref}}_k`$ is the path point at arc length $`s_k = s_0 + k\,v_{\mathrm{ref}}\Delta t`$ from the robot's projection $`s_0`$, with speed $`v_{\mathrm{ref}}`$, turn rate $`v_{\mathrm{ref}}\,\kappa(s_k)`$ and zero accelerations.

| | |
|---|---|
| Horizon | $`N = 30`$ stages of $`\Delta t = 0.1`$ s (3 s) |
| Weights | $`W = \mathrm{diag}(20, 20, 2, 2, 5, 0.2, 2, 0.2)`$, $`W_N = \mathrm{diag}(40, 40, 4, 4, 5)`$ |
| Obstacles | $`M = 6`$ per stage, the closest to the last predicted trajectory; walls as static pieces |
| Slack penalty | $`\rho_1 = 200`$, $`\rho_2 = 2000`$: the QP stays feasible when someone steps next to the robot |
| Limits (Burger) | $`v_{\max} = 0.20`$ m/s, $`\omega_{\max} = 2.0`$ rad/s, $`a_{\max} = 0.5`$ m/s², $`\alpha_{\max} = 3.0`$ rad/s² |
| Solver | SQP real-time iteration, Gauss-Newton, HPIPM with partial condensing, RK4 |

A track's velocity is used only after 3 detections and above 0.10 m/s ([`tracker.py`](../src/tb_nmpc/tb_nmpc/tracker.py)). Code: [`ocp.py`](../src/tb_nmpc/tb_nmpc/ocp.py).
