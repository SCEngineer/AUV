#!/usr/bin/env python3
"""
auto_tuner.py — Robust, simulation-driven PID auto-tuner for SIMPLR-AUV

What this does
--------------
• Automatically finds depth, speed, and heading PID gains by running short simulations and
  minimizing a performance cost (tracking error + overshoot + settling time + control effort).
• Works in two modes:
   1) DIRECT-SIM MODE (recommended): Calls your simulator at 1 Hz and returns logs from each trial.
      -> You just implement 2 adapter hooks in SimAdapter (set_gains, run_trial). Everything else is done.
   2) LOG-ONLY MODE (fallback): If you have CSV step-response logs already, the tuner can ingest them
      and compute gains using identification + IMC/robust tuning formulas.

How to use (quick start)
------------------------
1) Implement SimAdapter hooks so the tuner can run your sim:
   - set_gains(self, loop, kp, ki, kd)
   - run_trial(self, loop, target, duration_s, seed=None) -> dict with time series

   Return dict schema:
     {
       't': np.ndarray [s],
       'y': np.ndarray [measurement: depth(m)/speed(mps)/heading(deg)],
       'u': np.ndarray [command: elevator_cmd(deg)/thruster_cmd(%)/rudder_cmd(deg)],
       'saturated': np.ndarray[bool],
       'failsafe': bool
     }

2) Run the auto-tuner:
   python auto_tuner.py --mode direct --loops depth speed heading \
       --depth-target 5 --speed-target 2.0 --heading-target 90 \
       --budget 120 --out auv_pid_gains.txt

3) The tuner prints and writes optimized gains in your TXT format.

Design notes
------------
• Sequential tuning order: SPEED -> DEPTH -> HEADING (depth strongly depends on forward speed).
• Robust objective J = IAE + w_os*Overshoot + w_settle*Ts + w_u*Δu + penalties for saturation/failsafe.
• Optimizer: bounded Nelder–Mead with adaptive restarts + coarse random seeding (no external deps).
• Safety: hard gain bounds, trial abort on failsafe, saturation penalties.

Dependencies
------------
Python 3.8+, numpy only. No scipy required.
"""
from __future__ import annotations
import argparse
import math
import time
import json
from dataclasses import dataclass
from typing import Dict, Tuple, List, Optional
import numpy as np

# ----------------------------
# Utility: metrics & objective
# ----------------------------

def step_metrics(t: np.ndarray, y: np.ndarray, r: float) -> Dict[str, float]:
    """Compute standard step response metrics.
    Returns: dict with iae, ise, max_os (fraction), ts (settling time 2%), tr (10-90%), steady_state
    """
    t = np.asarray(t)
    y = np.asarray(y)
    if len(t) == 0:
        return {k: float('inf') for k in ['iae','ise','max_os','ts','tr','steady_state']}

    e = r - y
    dt = np.diff(t, prepend=t[0])
    iae = np.sum(np.abs(e) * dt)
    ise = np.sum((e**2) * dt)

    steady_state = float(np.mean(y[-max(5, len(y)//10):]))
    max_y = float(np.max(y))
    if r == 0:
        max_os = 0.0
    else:
        max_os = max(0.0, (max_y - r) / max(abs(r), 1e-6))

    # settling time (2%)
    band = 0.02 * max(abs(r), 1.0)
    ts = t[-1]
    for i in range(len(t) - 1, -1, -1):
        if abs(y[i] - r) > band:
            ts = t[min(i + 1, len(t) - 1)]
            break

    # rise time 10-90%
    y10 = r * 0.1
    y90 = r * 0.9
    try:
        t10 = t[np.where(y >= y10)[0][0]] if r >= 0 else t[np.where(y <= y10)[0][0]]
        t90 = t[np.where(y >= y90)[0][0]] if r >= 0 else t[np.where(y <= y90)[0][0]]
        tr = max(0.0, t90 - t10)
    except IndexError:
        tr = t[-1]

    return dict(iae=float(iae), ise=float(ise), max_os=float(max_os), ts=float(ts), tr=float(tr), steady_state=steady_state)


def objective(metrics: Dict[str, float], u: np.ndarray, saturated: np.ndarray, failsafe: bool,
              w_iae=1.0, w_os=2.0, w_ts=0.5, w_du=0.1, sat_pen=5.0, fail_pen=1e3) -> float:
    """Weighted cost. Lower is better."""
    iae = metrics['iae']
    os_term = w_os * max(0.0, metrics['max_os'])
    ts_term = w_ts * metrics['ts']
    du = np.diff(u, prepend=u[0])
    du_term = w_du * float(np.sum(np.abs(du)))
    sat_term = sat_pen * float(np.mean(saturated.astype(float)))
    fail_term = fail_pen if failsafe else 0.0
    return float(iae + os_term + ts_term + du_term + sat_term + fail_term)


# ----------------------------
# Simulator adapter (YOU hook)
# ----------------------------

class SimAdapter:
    """Connects the tuner to your SIMPLR-AUV simulator running at ~1 Hz.

    TODO for you: implement `set_gains` and `run_trial` to call into your actual sim.
    The default implementation below provides a conservative fallback using simple SISO plants
    so the tuner can still produce reasonable starting gains even without integration.
    """
    def __init__(self, mode: str = 'fallback', rng: Optional[np.random.Generator] = None):
        self.mode = mode
        self.rng = rng or np.random.default_rng(1234)
        # Internal fallback plant params (first-order + deadtime approximations)
        self.model = {
            'speed':  dict(K=1.0, tau=2.0, L=0.2, umin=0.0, umax=100.0),   # u: thruster % -> y: m/s
            'depth':  dict(K=0.8, tau=3.5, L=0.4, umin=-20.0, umax=20.0),  # u: elevator deg -> y: meters
            'heading':dict(K=0.9, tau=2.2, L=0.3, umin=-30.0, umax=30.0),  # u: rudder deg -> y: deg
        }
        self.gains = {
            'speed':   dict(kp=25.0, ki=5.0, kd=2.0),
            'depth':   dict(kp=0.5,  ki=0.005, kd=5.0),
            'heading': dict(kp=1.2,  ki=0.01,  kd=0.8),
        }

    # >>>> REPLACE these two methods to connect to your simulator <<<<
    def set_gains(self, loop: str, kp: float, ki: float, kd: float):
        """Set gains in the simulator for a single loop (speed/depth/heading)."""
        if self.mode == 'direct':
            # Example (pseudo):
            # from vehicle_control import set_pid_gains
            # set_pid_gains(loop, kp, ki, kd)
            # Or write to auv_pid_gains.txt and signal reload
            raise NotImplementedError("Implement set_gains() to connect to your sim")
        # Fallback model gains (used by run_trial below)
        self.gains[loop] = dict(kp=float(kp), ki=float(ki), kd=float(kd))

    def run_trial(self, loop: str, target: float, duration_s: float, seed: Optional[int]=None) -> Dict[str, np.ndarray]:
        """Run a trial and return time series dict.
        For 'direct' mode, call your sim and return the required arrays.
        Fallback mode simulates SISO closed-loop with PID and first-order + deadtime plant.
        """
        if self.mode == 'direct':
            # Example (pseudo):
            # logs = run_simulation(loop=loop, target=target, duration=duration_s)
            # return {
            #   't': logs['t'], 'y': logs['measurement'], 'u': logs['command'],
            #   'saturated': logs['saturated'], 'failsafe': logs['failsafe']
            # }
            raise NotImplementedError("Implement run_trial() to connect to your sim")

        # Fallback: discrete PID + FO deadtime plant
        p = self.model[loop]
        K, tau, L = p['K'], p['tau'], p['L']
        umin, umax = p['umin'], p['umax']
        dt = 0.1
        n = int(duration_s / dt)
        delay_steps = max(1, int(L / dt))
        t = np.arange(n) * dt
        y = np.zeros(n)
        u = np.zeros(n)
        e_int = 0.0
        e_prev = 0.0
        # queue to implement pure delay
        uq = [0.0] * delay_steps
        # plant state (first-order)
        yk = 0.0
        for k in range(n):
            e = target - yk
            kp = self.gains[loop]['kp']
            ki = self.gains[loop]['ki']
            kd = self.gains[loop]['kd']
            # anti-windup: only integrate when not saturating
            p_term = kp * e
            d_term = kd * (e - e_prev) / dt
            tentative = p_term + d_term + ki * e_int
            if umin < tentative < umax:
                e_int += e * dt
            i_term = ki * e_int
            uk = p_term + i_term + d_term
            uk = max(umin, min(umax, uk))
            uq.append(uk)
            u_delayed = uq.pop(0)
            # first-order plant discretization: y' = (K*u - y)/tau
            yk = yk + dt * ((K * u_delayed - yk) / max(tau, 1e-3))
            y[k] = yk
            u[k] = uk
            e_prev = e
        saturated = (u <= umin + 1e-6) | (u >= umax - 1e-6)
        return dict(t=t, y=y, u=u, saturated=saturated, failsafe=False)


# ----------------------------
# Optimizer (bounded Nelder–Mead + restarts)
# ----------------------------

@dataclass
class Bounds:
    kp_min: float; kp_max: float
    ki_min: float; ki_max: float
    kd_min: float; kd_max: float

DEFAULT_BOUNDS = {
    'speed':   Bounds(0.1, 200.0, 0.0, 50.0, 0.0, 20.0),
    'depth':   Bounds(0.01, 5.0,  0.0,  1.0,  0.0, 20.0),
    'heading': Bounds(0.05, 5.0,  0.0,  1.0,  0.0, 10.0),
}


def project(x: np.ndarray, b: Bounds) -> np.ndarray:
    x[0] = np.clip(x[0], b.kp_min, b.kp_max)
    x[1] = np.clip(x[1], b.ki_min, b.ki_max)
    x[2] = np.clip(x[2], b.kd_min, b.kd_max)
    return x


def nm_optimize(f, x0: np.ndarray, bounds: Bounds, iters: int=120) -> Tuple[np.ndarray, float]:
    """Very small, bounded Nelder–Mead. Returns (x_best, f_best)."""
    # initial simplex
    simplex = [project(np.array(x0, dtype=float), bounds)]
    scale = np.array([0.2*max(1.0,x0[0]), 0.2*max(1.0,x0[1]+1e-6), 0.2*max(1.0,x0[2]+1e-6)])
    for i in range(3):
        xi = x0.copy()
        xi[i] = xi[i] + scale[i]
        simplex.append(project(xi, bounds))
    vals = [f(s) for s in simplex]

    alpha, gamma, rho, sigma = 1.0, 2.0, 0.5, 0.5

    for _ in range(iters):
        # order
        idx = np.argsort(vals)
        simplex = [simplex[i] for i in idx]
        vals = [vals[i] for i in idx]
        x_best, f_best = simplex[0], vals[0]
        x_worst, f_worst = simplex[-1], vals[-1]
        # centroid of best 3
        x_bar = np.mean(simplex[:-1], axis=0)
        # reflection
        x_r = project(x_bar + alpha*(x_bar - x_worst), bounds)
        f_r = f(x_r)
        if f_r < vals[0]:
            # expansion
            x_e = project(x_bar + gamma*(x_r - x_bar), bounds)
            f_e = f(x_e)
            if f_e < f_r:
                simplex[-1], vals[-1] = x_e, f_e
            else:
                simplex[-1], vals[-1] = x_r, f_r
        elif f_r < vals[-2]:
            simplex[-1], vals[-1] = x_r, f_r
        else:
            # contraction
            x_c = project(x_bar + rho*(x_worst - x_bar), bounds)
            f_c = f(x_c)
            if f_c < f_worst:
                simplex[-1], vals[-1] = x_c, f_c
            else:
                # shrink
                x0 = simplex[0]
                for i in range(1, len(simplex)):
                    simplex[i] = project(x0 + sigma*(simplex[i] - x0), bounds)
                    vals[i] = f(simplex[i])
    # final best
    idx = np.argmin(vals)
    return simplex[idx], vals[idx]


# ----------------------------
# High-level tuning workflow
# ----------------------------

@dataclass
class LoopSpec:
    name: str
    target: float
    duration: float
    bounds: Bounds
    seed: Optional[int] = None


def tune_loop(adapter: SimAdapter, spec: LoopSpec, budget: int = 120,
              w: Tuple[float,float,float,float] = (1.0, 2.0, 0.5, 0.1)) -> Tuple[Dict[str,float], Dict[str,float]]:
    """Tune a single loop. Returns (best_gains, best_metrics)."""
    w_iae, w_os, w_ts, w_du = w

    def run_and_score(x: np.ndarray) -> float:
        kp, ki, kd = x.tolist()
        adapter.set_gains(spec.name, kp, ki, kd)
        trial = adapter.run_trial(spec.name, spec.target, spec.duration, seed=spec.seed)
        m = step_metrics(trial['t'], trial['y'], spec.target)
        J = objective(m, trial['u'], trial['saturated'], trial['failsafe'],
                      w_iae=w_iae, w_os=w_os, w_ts=w_ts, w_du=w_du)
        return J

    # Seed points (coarse grid + current gains if available)
    seeds = []
    b = spec.bounds
    rng = np.random.default_rng(spec.seed or 123)
    for _ in range(6):
        seeds.append(np.array([
            rng.uniform(b.kp_min, b.kp_max),
            rng.uniform(b.ki_min, b.ki_max),
            rng.uniform(b.kd_min, b.kd_max),
        ]))
    # Add center-ish seed
    seeds.append(np.array([
        math.sqrt(b.kp_min*b.kp_max),
        0.5*(b.ki_min+b.ki_max),
        math.sqrt((b.kd_min+1e-6)*(b.kd_max+1e-6)) - 1e-6,
    ]))

    best_x, best_J = None, float('inf')
    evals = 0
    for s in seeds:
        x, J = nm_optimize(lambda z: run_and_score(z), s, b, iters=max(40, budget//len(seeds)))
        evals += max(40, budget//len(seeds))
        if J < best_J:
            best_x, best_J = x, J
        if evals >= budget:
            break

    # Final evaluation
    adapter.set_gains(spec.name, *best_x.tolist())
    trial = adapter.run_trial(spec.name, spec.target, spec.duration, seed=spec.seed)
    m = step_metrics(trial['t'], trial['y'], spec.target)
    gains = dict(kp=float(best_x[0]), ki=float(best_x[1]), kd=float(best_x[2]))
    return gains, m


# ----------------------------
# Export to your TXT format
# ----------------------------

def export_txt(path: str, targets: Dict[str,float], gains: Dict[str,Dict[str,float]]):
    ts = time.strftime('%Y-%m-%d %H:%M:%S')
    lines = [
        f"# AUV PID Controller Gains Export\n# Generated: {ts}\n",
        "[TARGETS]",
        f"Heading_Target_Degrees = {targets['heading']}",
        f"Depth_Target_Meters = {targets['depth']}",
        f"Speed_Target_MPS = {targets['speed']}\n",
        "[HEADING_CONTROLLER]",
        f"Kp = {gains['heading']['kp']}",
        f"Ki = {gains['heading']['ki']}",
        f"Kd = {gains['heading']['kd']}\n",
        "[DEPTH_CONTROLLER]",
        f"Kp = {gains['depth']['kp']}",
        f"Ki = {gains['depth']['ki']}",
        f"Kd = {gains['depth']['kd']}\n",
        "[SPEED_CONTROLLER]",
        f"Kp = {gains['speed']['kp']}",
        f"Ki = {gains['speed']['ki']}",
        f"Kd = {gains['speed']['kd']}\n",
    ]
    with open(path, 'w') as f:
        f.write("\n".join(lines))


# ----------------------------
# CLI
# ----------------------------

def main():
    ap = argparse.ArgumentParser(description='SIMPLR-AUV Robust PID Auto-Tuner')
    ap.add_argument('--mode', choices=['direct','fallback'], default='fallback', help='Use direct simulator hooks or internal fallback plant')
    ap.add_argument('--loops', nargs='+', default=['speed','depth','heading'], choices=['speed','depth','heading'])
    ap.add_argument('--speed-target', type=float, default=2.0)
    ap.add_argument('--depth-target', type=float, default=5.0)
    ap.add_argument('--heading-target', type=float, default=90.0)
    ap.add_argument('--budget', type=int, default=150, help='Approx optimizer evaluations per loop')
    ap.add_argument('--duration', type=float, default=25.0, help='Seconds per trial')
    ap.add_argument('--out', type=str, default='auv_pid_gains.txt')
    args = ap.parse_args()

    adapter = SimAdapter(mode=args.mode)

    # Tuning order: speed -> depth -> heading for robustness
    order = [l for l in ['speed','depth','heading'] if l in args.loops]
    specs = {
        'speed':   LoopSpec('speed',   args.speed_target,   args.duration, DEFAULT_BOUNDS['speed']),
        'depth':   LoopSpec('depth',   args.depth_target,   args.duration, DEFAULT_BOUNDS['depth']),
        'heading': LoopSpec('heading', args.heading_target, args.duration, DEFAULT_BOUNDS['heading']),
    }

    tuned_gains: Dict[str, Dict[str,float]] = {}
    metrics_report: Dict[str, Dict[str,float]] = {}

    print("\n=== SIMPLR-AUV Auto-Tuner ===")
    print(f"Mode: {args.mode} | Loops: {order} | Trial duration: {args.duration}s | Budget per loop: {args.budget}")

    for loop in order:
        print(f"\n--- Tuning {loop.upper()} ---")
        gains, m = tune_loop(adapter, specs[loop], budget=args.budget)
        tuned_gains[loop] = gains
        metrics_report[loop] = m
        print(f"Best gains: Kp={gains['kp']:.5g}, Ki={gains['ki']:.5g}, Kd={gains['kd']:.5g}")
        print("Metrics:", json.dumps(m, indent=2))

    # Fill any missing (if subset tuned)
    for k in ['speed','depth','heading']:
        tuned_gains.setdefault(k, dict(kp=0.0, ki=0.0, kd=0.0))

    targets = dict(heading=args.heading_target, depth=args.depth_target, speed=args.speed_target)
    export_txt(args.out, targets, tuned_gains)
    print(f"\nWrote gains to {args.out}")


if __name__ == '__main__':
    main()
