"""Nonlinear MPC for a differential-drive robot, generated with acados.

Model: unicycle kinematics with the velocities as states, so that acceleration limits are
hard constraints and the command is smooth:
    state  x = [px, py, theta, v, omega]
    input  u = [a, alpha]                     (linear and angular acceleration)
    px' = v cos theta,  py' = v sin theta,  theta' = omega,  v' = a,  omega' = alpha

Cost: nonlinear least squares on [px, py, cos theta, sin theta, v, omega, a, alpha] against the
reference (heading through cos/sin, so no angle wrapping).

Obstacles: M circles per stage, given as parameters p_k = [ox_1, oy_1, r_1, ..., ox_M, oy_M, r_M].
They are the tracker's predictions at the stage time, so the constraints vary along the
horizon. Each one is  sqrt((px-ox)^2 + (py-oy)^2) - r >= 0,  softened with an L1 + L2 slack
penalty so the QP stays feasible when a person steps right next to the robot.

Solver: SQP real-time iteration (one Gauss-Newton QP per control step), HPIPM with partial
condensing, explicit RK4.
"""
from __future__ import annotations

import hashlib
import os
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

NX, NU, NY = 5, 2, 8
FAR = 1.0e3  # position of unused obstacle slots


@dataclass
class OcpConfig:
    """Settings of the optimal control problem: horizon, obstacle slots, limits, weights."""

    N: int = 30                 # stages
    dt: float = 0.1             # [s] stage length (3 s horizon)
    M: int = 6                  # obstacle slots per stage
    v_min: float = 0.0
    v_max: float = 0.20
    w_max: float = 2.0
    a_max: float = 0.5
    alpha_max: float = 3.0
    # Weights on [px, py, cos, sin, v, omega, a, alpha]; terminal on the first five.
    W: list = field(default_factory=lambda: [20.0, 20.0, 2.0, 2.0, 5.0, 0.2, 2.0, 0.2])
    W_e: list = field(default_factory=lambda: [40.0, 40.0, 4.0, 4.0, 5.0])
    slack_l1: float = 200.0     # obstacle slack penalty, linear and quadratic
    slack_l2: float = 2000.0

    def key(self) -> str:
        """Hash of what is baked into the generated code (dimensions)."""
        return hashlib.sha1(f"{self.N}-{self.dt}-{self.M}".encode()).hexdigest()[:10]


def build_ocp(cfg: OcpConfig):
    """Describe the problem to acados (model, cost, constraints, solver options)."""
    import casadi as ca
    from acados_template import AcadosModel, AcadosOcp

    px, py, th, v, w = (ca.SX.sym(n) for n in ("px", "py", "theta", "v", "omega"))
    a, al = ca.SX.sym("a"), ca.SX.sym("alpha")
    x = ca.vertcat(px, py, th, v, w)
    u = ca.vertcat(a, al)
    p = ca.SX.sym("p", 3 * cfg.M)

    model = AcadosModel()
    model.name = "unicycle"
    model.x, model.u, model.p = x, u, p
    # Continuous dynamics x' = f(x, u); acados discretises it over dt with RK4 (options below)
    model.f_expl_expr = ca.vertcat(v * ca.cos(th), v * ca.sin(th), w, a, al)
    model.xdot = ca.SX.sym("xdot", NX)
    model.f_impl_expr = model.xdot - model.f_expl_expr
    # y compared with yref at every stage (weights W), y_e at the last one (W_e, no input there)
    model.cost_y_expr = ca.vertcat(px, py, ca.cos(th), ca.sin(th), v, w, a, al)
    model.cost_y_expr_e = ca.vertcat(px, py, ca.cos(th), ca.sin(th), v)
    # One constraint per obstacle slot: distance to its centre minus its radius >= 0
    h = ca.vertcat(*[ca.sqrt((px - p[3 * j]) ** 2 + (py - p[3 * j + 1]) ** 2 + 1e-6) - p[3 * j + 2]
                     for j in range(cfg.M)])
    model.con_h_expr = h
    model.con_h_expr_e = h

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = cfg.N
    ocp.solver_options.tf = cfg.N * cfg.dt
    ocp.parameter_values = np.tile([FAR, FAR, 0.0], cfg.M)

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.W = np.diag(cfg.W)
    ocp.cost.W_e = np.diag(cfg.W_e)
    ocp.cost.yref = np.zeros(NY)
    ocp.cost.yref_e = np.zeros(5)

    ocp.constraints.x0 = np.zeros(NX)
    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lbu = np.array([-cfg.a_max, -cfg.alpha_max])
    ocp.constraints.ubu = np.array([cfg.a_max, cfg.alpha_max])
    ocp.constraints.idxbx = np.array([3, 4])
    ocp.constraints.lbx = np.array([cfg.v_min, -cfg.w_max])
    ocp.constraints.ubx = np.array([cfg.v_max, cfg.w_max])
    ocp.constraints.idxbx_e = np.array([3, 4])
    ocp.constraints.lbx_e = np.array([cfg.v_min, -cfg.w_max])
    ocp.constraints.ubx_e = np.array([cfg.v_max, cfg.w_max])

    # Soft obstacle constraints: a violation s costs slack_l1 * s + slack_l2 * s^2 instead of
    # making the problem infeasible
    big = 1.0e4
    for suffix in ("", "_e"):
        setattr(ocp.constraints, "lh" + suffix, np.zeros(cfg.M))
        setattr(ocp.constraints, "uh" + suffix, np.full(cfg.M, big))
        setattr(ocp.constraints, "idxsh" + suffix, np.arange(cfg.M))
        setattr(ocp.cost, "zl" + suffix, np.full(cfg.M, cfg.slack_l1))
        setattr(ocp.cost, "zu" + suffix, np.full(cfg.M, cfg.slack_l1))
        setattr(ocp.cost, "Zl" + suffix, np.full(cfg.M, cfg.slack_l2))
        setattr(ocp.cost, "Zu" + suffix, np.full(cfg.M, cfg.slack_l2))

    o = ocp.solver_options
    o.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    o.hessian_approx = "GAUSS_NEWTON"
    o.integrator_type = "ERK"
    o.sim_method_num_stages = 4
    o.sim_method_num_steps = 1
    o.nlp_solver_type = "SQP_RTI"
    o.qp_solver_iter_max = 50
    o.qp_solver_cond_N = max(1, cfg.N // 5)
    return ocp


class Nmpc:
    """Thin wrapper around the generated solver: set reference and obstacles, solve, read back."""

    def __init__(self, cfg: OcpConfig, code_dir: str | None = None, verbose: bool = False):
        """Generate and compile the solver, or reuse it from ~/.cache/tb_nmpc if N, dt, M are unchanged."""
        from acados_template import AcadosOcpSolver

        self.cfg = cfg
        base = Path(code_dir or os.path.expanduser("~/.cache/tb_nmpc")) / f"acados_{cfg.key()}"
        base.mkdir(parents=True, exist_ok=True)
        ocp = build_ocp(cfg)
        ocp.code_export_directory = str(base / "c_generated_code")
        cwd = os.getcwd()
        os.chdir(base)  # acados builds relative to the working directory
        try:
            self.solver = AcadosOcpSolver(ocp, json_file=str(base / "unicycle_ocp.json"), verbose=verbose)
        finally:
            os.chdir(cwd)
        self.X = np.zeros((cfg.N + 1, NX))
        self.U = np.zeros((cfg.N, NU))
        self.initialized = False

    def set_limits(self, v_min, v_max, w_max, a_max, alpha_max) -> None:
        """Change the bounds on (a, alpha) and (v, omega) at run time, without recompiling."""
        s = self.solver
        for k in range(self.cfg.N):
            s.constraints_set(k, "lbu", np.array([-a_max, -alpha_max]))
            s.constraints_set(k, "ubu", np.array([a_max, alpha_max]))
        for k in range(1, self.cfg.N + 1):
            s.constraints_set(k, "lbx", np.array([v_min, -w_max]))
            s.constraints_set(k, "ubx", np.array([v_max, w_max]))

    def solve(self, x0: np.ndarray, ref: np.ndarray, obstacles: np.ndarray, rti_iters: int = 1):
        """One control step.

        x0: (5,) current state; ref: (N+1, 5) rows (x, y, theta, v, omega);
        obstacles: (N+1, M, 3) predicted circles, radii already inflated by robot radius + margin.
        Returns (status, solve time [s] summed over the iterations).
        """
        s, N = self.solver, self.cfg.N
        if not self.initialized:
            # Cold start: roll the reference out as the initial guess.
            self.X[:] = ref
            self.X[0] = x0
            self.U[:] = 0.0
            self.initialized = True
        else:
            # Warm start: shift by one stage (the control period equals dt).
            self.X[:-1] = self.X[1:]
            self.U[:-1] = self.U[1:]
        self.X[0] = x0
        # Keep the warm start's heading on the same branch as the measured heading.
        self.X[:, 2] += np.round((x0[2] - self.X[1, 2]) / (2 * np.pi)) * 2 * np.pi
        s.set(0, "lbx", x0)
        s.set(0, "ubx", x0)
        for k in range(N + 1):
            r = ref[k]
            if k < N:
                s.set(k, "yref", np.array([r[0], r[1], np.cos(r[2]), np.sin(r[2]), r[3], r[4], 0.0, 0.0]))
            else:
                s.set(k, "yref", np.array([r[0], r[1], np.cos(r[2]), np.sin(r[2]), r[3]]))
            s.set(k, "p", obstacles[k].reshape(-1))
            s.set(k, "x", self.X[k])
            if k < N:
                s.set(k, "u", self.U[k])
        status, t = 0, 0.0
        for _ in range(max(1, rti_iters)):
            status = s.solve()
            t += s.get_stats("time_tot")
        for k in range(N + 1):
            self.X[k] = s.get(k, "x")
            if k < N:
                self.U[k] = s.get(k, "u")
        return status, t

    def reset(self) -> None:
        """Cold start at the next solve (new path, or after a solver failure)."""
        self.initialized = False
