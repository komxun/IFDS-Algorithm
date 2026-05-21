# Technical Report: IFDS Dynamic Autorouting with SE(3) Geometric Quadrotor Control

**Author:** Komsun Tamanakijprasart  
**Date:** May 2026  
**Repository:** `IFDS-Algorithm`

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [IFDS Path Planning](#2-ifds-path-planning)
3. [Quadrotor UAV Dynamics Model](#3-quadrotor-uav-dynamics-model)
4. [SE(3) Geometric Tracking Controller](#4-se3-geometric-tracking-controller)
5. [Reference Trajectory Generation](#5-reference-trajectory-generation)
6. [Numerical Integration](#6-numerical-integration)
7. [Integration Architecture](#7-integration-architecture)
8. [Hold-Position (Hover) Mode](#8-hold-position-hover-mode)
9. [Telemetry Logging and Visualisation](#9-telemetry-logging-and-visualisation)
10. [Software File Reference](#10-software-file-reference)
11. [UAV and Controller Parameters](#11-uav-and-controller-parameters)
12. [Key Design Decisions](#12-key-design-decisions)
13. [References](#13-references)

---

## 1. System Overview

This simulation implements a 3-D dynamic obstacle-avoidance framework for an autonomous quadrotor UAV. The system couples two subsystems operating at different timescales:

| Layer | Algorithm | File | Rate |
|-------|-----------|------|------|
| **Path planning** | IFDS (Iterative Forward–Backward Dynamic System) | `IFDS.m` | 10 Hz (`dt = 0.1 s`) |
| **Flight control** | SE(3) geometric tracking + 18-state rigid-body dynamics | `SE3Track.m` | 100 Hz (`Ts = 0.01 s`) |

At each outer planning step the IFDS algorithm produces a collision-free waypoint sequence. The SE(3) tracker then physically flies the UAV along each waypoint segment, respecting full translational and rotational dynamics, until the allocated time budget (`dt_traj = 1 s`) is consumed. If IFDS cannot find a path, the UAV enters an SE(3) hover mode (`hold_position.m`) at its current location.

---

## 2. IFDS Path Planning

### 2.1 Algorithm

The Iterative Forward–Backward Dynamic System (IFDS) is a modulation-based reactive planner that deforms a nominal straight-line velocity field to avoid obstacles. At each time step the planner integrates

```
Ẇ = M(W) · ṽ(W)
```

where `ṽ` is the nominal field pointing toward the goal, and `M(W)` is a modulation matrix constructed from obstacle geometry (superquadric representations) and, optionally, a weather-hazard field weighted by parameter `k`.

### 2.2 Weather Constraint

When `k ≠ 0` the obstacle boundary function `Γ` is augmented by a spatially varying weather map `ω(x,y)`:

```
Γ'(x,y,z) = Γ − k · (exp((B_L − ω)/(B_L − B_U) · ln((Γ−1)/k + 1)) − 1)
```

with gradient components `∂Γ'/∂x`, `∂Γ'/∂y`, `∂Γ'/∂z` computed analytically. The parameters `B_U ∈ (0,1]` and `B_L ∈ [0, B_U)` set the occupancy thresholds of the weather field.

### 2.3 Output

IFDS outputs a sequence of 3-D waypoints `Paths{rt}` (3 × N matrix) representing the planned path from the UAV's current position to the goal, re-computed at every outer iteration `rt`.

---

## 3. Quadrotor UAV Dynamics Model

### 3.1 Coordinate Frames

| Frame | Symbol | Description |
|-------|--------|-------------|
| Inertial | {e₁, e₂, e₃} | Fixed world frame, e₃ pointing upward |
| Body | {b₁, b₂, b₃} | Fixed to UAV centre of mass, b₃ aligned with thrust axis |

The rotation matrix **R** ∈ SO(3) maps body-frame coordinates to inertial-frame coordinates.

### 3.2 State Vector

The UAV is modelled as a rigid body with 18 states:

```
x = [p ∈ R³;  v ∈ R³;  vec(R) ∈ R⁹;  Ω ∈ R³]
```

| Symbol | Size | Units | Description |
|--------|------|-------|-------------|
| **p** | 3×1 | m | Inertial position |
| **v** | 3×1 | m/s | Inertial (world-frame) velocity |
| **R** | 3×3 | — | Body-to-inertial rotation matrix, R ∈ SO(3) |
| **Ω** | 3×1 | rad/s | Angular velocity expressed in the **body** frame |

### 3.3 Equations of Motion

The continuous-time dynamics follow Lee et al. (2010), equations (2)–(5):

**Translational kinematics and dynamics:**
```
ṗ = v                                           (1)
mv̇ = mg e₃ − f R e₃                            (2)
```

**Rotational kinematics and dynamics:**
```
Ṙ = R Ω̂                                         (3)
J Ω̇ = M − Ω × J Ω                              (4)
```

where:

| Symbol | Value | Description |
|--------|-------|-------------|
| m | 4.34 kg | Total UAV mass |
| g | 9.81 m/s² | Gravitational acceleration |
| **J** | diag(0.0820, 0.0845, 0.1377) kg·m² | Principal inertia tensor |
| f | scalar [N] | Collective thrust magnitude (along body b₃ axis) |
| **M** | 3×1 [N·m] | Net moment vector in body frame |
| Ω̂ | 3×3 | Skew-symmetric matrix (`hat(Ω)`) |
| e₃ | [0; 0; 1] | Third axis of inertial frame |

**Note on gravity sign convention.** Equation (2) has `+mg e₃` because the inertial z-axis points upward. Thrust `f R e₃` acts upward along the body b₃ axis and must be positive to counteract gravity.

### 3.4 Control Inputs

The two physical inputs to the dynamics are:

- **f ∈ R** — collective (total) thrust [N], acts along body b₃ = R e₃
- **M ∈ R³** — body-frame moment (roll, pitch, yaw torque) [N·m]

These are allocated to individual rotor forces via the **mixing matrix**:

```
[f₁; f₂; f₃; f₄] = P.Mix · [f; M]
```

where P.Mix = inv([1 1 1 1; 0 −d 0 d; d 0 −d 0; −c_τf c_τf −c_τf c_τf]) with arm length d = 0.315 m and rotor drag/thrust ratio c_τf = 8.004 × 10⁻³ m.

### 3.5 Modelling Assumptions

The following aerodynamic effects are **deliberately omitted**, consistent with the standard Lee 2010/2011 academic quadrotor model:

- **No aerodynamic drag or lift.** The model operates in vacuum. For low-speed indoor flight (v < 5 m/s) this is acceptable; at higher speeds or outdoors, a drag term `F_drag = −½ρ Cd S ‖v‖² v̂` should be added to equation (2).
- **No rotor gyroscopic effects.** Rotor angular momentum is not modelled.
- **No flexible body or motor dynamics.** Rotors are assumed ideal with instantaneous thrust response.
- **No angle-of-attack effects.** Quadrotors generate thrust primarily from rotor disc area (not wing lift), so `Cl(α)`, `Cd(α)` functions do not apply in the traditional fixed-wing sense.

---

## 4. SE(3) Geometric Tracking Controller

The controller is ported directly from Lee et al. (2010/2011) (arXiv:1003.2005v4) as implemented in `se3quad/matlab/controller.m`.

### 4.1 Error Functions

Given desired position **x**_d(t) and desired body-1 direction **b**_{1d}:

```
e_x = p − x_d                         (position error)
e_v = v − ẋ_d                         (velocity error)
e_a = v̇ − ẍ_d                         (acceleration error, via dirty derivative)
e_j = v̈ − x⃛_d                         (jerk error, via dirty derivative)
```

Attitude errors (Lee 2010, Eq. 10–11):
```
e_R = ½ vee(R_c^T R − R^T R_c)        (SO(3) attitude error)
e_Ω = Ω − R^T R_c Ω_c                 (angular velocity error)
```

The scalar **attitude error function** (Lyapunov-like) is:
```
Ψ(R, R_c) = ½ tr(I − R_c^T R) ∈ [0, 2)
```
Ψ = 0 at perfect attitude tracking; Ψ approaching 2 indicates near-maximum misalignment.

### 4.2 Translational Control — Thrust Direction

Define the auxiliary vector (Lee 2010, Eq. 19):

```
A = −k_x e_x − k_v e_v − m g e₃ + m ẍ_d
```

The collective thrust magnitude is:

```
f = −A · (R e₃)                        (5)
```

and the desired body-3 axis is:

```
b₃_c = −A / ‖A‖                        (6)
```

### 4.3 Desired Attitude R_c

Given b₃_c from (6) and the desired heading direction **b**_{1d} (projected onto the plane orthogonal to b₃_c):

```
C    = b₃_c × b₁_d
b₁_c = −(b₃_c × C) / ‖C‖
b₂_c =  C / ‖C‖
R_c  = [b₁_c  b₂_c  b₃_c]             (7)
```

**b**_{1d} is set from the horizontal heading of each IFDS path segment: **b**_{1d} = [cos ψ_d; sin ψ_d; 0] where ψ_d = atan2(d̂_y, d̂_x).

### 4.4 Feedforward Angular Velocity and Acceleration

Time derivatives of R_c are computed analytically from A, Ȧ, Ä (Lee 2011 Appendix F):

```
ḃ₃_c = −Ȧ/‖A‖ + (A·Ȧ/‖A‖³) A
Ċ    = ḃ₃_c × b₁_d + b₃_c × ḃ₁_d
...
Ω_c      = vee(R_c^T Ṙ_c)             (commanded body rate)
Ω̇_c      = vee(R_c^T R̈_c − Ω̂_c²)   (commanded angular acceleration)
```

These feedforward terms are computed from numerical dirty-derivative estimates of v (actual velocity) — see Section 4.6.

### 4.5 Moment Control

```
M = −k_R e_R − k_Ω e_Ω + Ω × J Ω − J(Ω̂ R^T R_c Ω_c − R^T R_c Ω̇_c)    (8)
```

The first two terms are proportional–derivative feedback; the last two are Coriolis compensation and angular feedforward.

### 4.6 Dirty-Derivative Filters

The feed-forward terms `e_a`, `e_j` require `v̇` and `v̈`. These are estimated using a band-limited first-order filter (`DirtyDerivative.m`) applied to the measured velocity v:

```
Transfer function:  P(s) = s / (τ s + 1)
Discrete form:      ẋ[k] = a₁ ẋ[k−1] + a₂ (x[k] − x[k−1])
                    a₁ = (2τ − Ts) / (2τ + Ts)
                    a₂ = 2 / (2τ + Ts)
```

with τ = 0.05 s (first derivative) and τ = 0.50 s (second derivative), Ts = 0.01 s.

### 4.7 Control Gains

| Gain | Value | Role |
|------|-------|------|
| k_x | 4m = 17.36 N/m | Position stiffness |
| k_v | 5.6m = 24.30 N·s/m | Velocity damping |
| k_R | 8.81 N·m/rad | Attitude stiffness |
| k_Ω | 2.54 N·m·s/rad | Angular rate damping |

The position-loop natural frequency is ωn = √(k_x/m) = **2 rad/s** (period ≈ 3.1 s).

---

## 5. Reference Trajectory Generation

Within each IFDS path segment (Wi → Wf), the reference position moves linearly along the segment at the cruise speed V_ref = C:

```
x_d(t) = Wi + min(V_ref · t, ‖Wf − Wi‖) · d̂
ẋ_d    = V_ref · d̂    (while s < ‖Wf − Wi‖)
ẍ_d = x⃛_d = x⃝_d = 0
```

where d̂ = (Wf − Wi) / ‖Wf − Wi‖. Similarly, **b**_{1d} is constant within a segment (the segment's horizontal heading), so ḃ_{1d} = b̈_{1d} = 0.

This analytic parameterisation avoids differentiating a noisy discrete waypoint stream; dirty-derivative filters are reserved for the **actual** velocity v only.

---

## 6. Numerical Integration

Equations (1)–(4) are integrated with a **4th-order Runge–Kutta (RK4)** method at step size Ts = 0.01 s. The controls (f, M) are held constant over each step (zero-order hold).

After each RK4 step, the rotation matrix R is re-projected onto SO(3) via **SVD**:

```
[U, Σ, V] = svd(R)
R ← U V^T          (det = +1 branch enforced)
```

This prevents the algebraic drift R^T R → I from degrading over long simulations.

---

## 7. Integration Architecture

### 7.1 Timescale Separation

```
Outer loop  (main.m, rt = 1 … rtsim)
│   Step 1:  IFDS plans path Paths{rt}  from current position at 10 Hz
│   Step 2:  Inner tracking loop for dt_traj = 1 s
│   │
│   Inner loop  (SE3Track, j = waypoint index)
│   │   SE3Track tracks segment Wi → Wf at 100 Hz (Ts = 0.01 s)
│   │   until: UAV crosses normal plane through Wf, OR dt_traj exhausted
│   │   Returns: final state (p, v, R, Ω), dense pos_hist, vm, logger
│   │
│   State hand-off:   state (p, v, R, Ω) passed to next SE3Track call
│   Filter hand-off:  DirtyDerivative handles persist (class handle semantics)
│
│   If IFDS fails:  hold_position(state, filters, dt_traj, P, logger)
│
│   Bookkeeping:  pos, vhist, traj{rt} updated; timer recorded
```

### 7.2 Segment Exit Condition

A segment ends when:

- **Normal-plane crossing:** `(Wf − Wi) · (p − Wf) ≥ 0` — UAV has passed the plane through Wf perpendicular to the segment direction.
- **Time budget exhausted:** inner loop time `t ≥ dt_budget = dt_traj − dtcum`.
- **Degenerate segment:** ‖Wf − Wi‖ < 10⁻⁶ m (skipped immediately).

### 7.3 State Continuity

Unlike `CCA3D_2.m` which re-initialised Euler angles each call, the full SE(3) state struct and `DirtyDerivative` filter handles are passed by reference across every segment and every outer IFDS iteration. This ensures:

- **No velocity jumps** at segment boundaries
- **No attitude discontinuities** at IFDS re-plans  
- **Smooth filter transients** — dirty derivatives accumulate consistent velocity history

---

## 8. Hold-Position (Hover) Mode

When IFDS cannot find a collision-free path, the UAV enters hover mode (`hold_position.m`). The SE(3) controller is called with:

```
x_d    = p  (freeze at current position)
ẋ_d   = 0
ẍ_d = x⃛_d = x⃝_d = 0
b₁_d   = [1; 0; 0]  (head north, arbitrary but fixed)
```

The controller commands `f ≈ mg` (gravity compensation) and drives `e_x, e_v, e_R, e_Ω → 0`. In perfect hover equilibrium: R = I, Ω = 0, v = 0, f = mg = 42.6 N per rotor ≈ 10.6 N.

---

## 9. Telemetry Logging and Visualisation

### 9.1 Logger Struct

At every controller step, `SE3Track.m` and `hold_position.m` append to a `logger` struct:

| Field | Size | Content |
|-------|------|---------|
| `logger.t` | 1×N | Global simulation time [s] |
| `logger.x` | 3×N | Actual inertial position [m] |
| `logger.xd` | 3×N | Desired inertial position [m] |
| `logger.v` | 3×N | Actual inertial velocity [m/s] |
| `logger.vd` | 3×N | Desired velocity [m/s] |
| `logger.Omega` | 3×N | Actual body angular rate [rad/s] |
| `logger.Omegac` | 3×N | Commanded angular rate Ω_c [rad/s] |
| `logger.Psi` | 1×N | SO(3) error function Ψ |
| `logger.f` | 1×N | Collective thrust [N] |
| `logger.M` | 3×N | Body-frame moment [N·m] |
| `logger.deltaF` | 4×N | Per-rotor forces [N] |

### 9.2 Batch Plotter (`se3_plot.m`)

After simulation, `se3_plot(logger, P)` produces four figures:

| Figure | Content |
|--------|---------|
| **2** | Translational states: x,y,z and v_x,v_y,v_z (actual vs desired) |
| **3** | Rotational states: Ω_x,Ω_y,Ω_z (actual vs commanded) and Ψ(t) |
| **4** | Actuators: per-rotor forces f₁–f₄, total thrust f, moments M_x,M_y,M_z |
| **5** | Tracking error norms: ‖p − p_d‖, ‖v − v_d‖, Ψ |

---

## 10. Software File Reference

| File | Role |
|------|------|
| `main.m` | Top-level simulation: IFDS loop, SE3Track calls, plotting |
| `IFDS.m` | IFDS path planning algorithm |
| `SE3Track.m` | SE(3) geometric tracker — controller + dynamics + telemetry |
| `hold_position.m` | SE(3) hover controller for path-not-found intervals |
| `se3_plot.m` | Offline batch plotter for telemetry logger |
| `DirtyDerivative.m` | Band-limited filtered differentiator (handle class) |
| `hat.m` | R³ → so(3) skew-symmetric map |
| `vee.m` | so(3) → R³ inverse of hat |
| `CCA3D_2.m` | Legacy carrot-chasing tracker (retained, no longer called) |

---

## 11. UAV and Controller Parameters

All physical and control parameters are set in `main.m` and match exactly those in `se3quad/matlab/param.m`:

```matlab
P.Ts      = 0.01;              % [s]      Controller / integrator step
P.gravity = 9.81;              % [m/s²]   Gravitational acceleration
P.mass    = 4.34;              % [kg]     Total mass
P.Jxx     = 0.0820;            % [kg·m²]  Roll inertia
P.Jyy     = 0.0845;            % [kg·m²]  Pitch inertia
P.Jzz     = 0.1377;            % [kg·m²]  Yaw inertia
P.tau     = 0.05;              % [s]      Dirty-derivative filter time constant
P.kx      = 4   * P.mass;     % [N/m]    Position gain  → 17.36
P.kv      = 5.6 * P.mass;     % [N·s/m]  Velocity gain  → 24.30
P.kR      = 8.81;              % [N·m/rad] Attitude gain
P.kOmega  = 2.54;              % [N·m·s]  Angular rate gain
P.d       = 0.315;             % [m]      CoM-to-rotor arm length
P.c_tauf  = 8.004e-3;          % [m]      Rotor drag/thrust ratio
```

**Hover equilibrium check:**  
At steady hover: f = mg = 4.34 × 9.81 = **42.57 N**, distributed equally to four rotors as **10.64 N** each.

---

## 12. Key Design Decisions

### 12.1 Analytic vs Filtered Reference Derivatives

The se3quad Simulink controller computes `ẋ_d, ẍ_d, x⃛_d, x⃝_d` via cascaded dirty-derivative filters on the commanded trajectory. In the IFDS framework, the desired trajectory within each segment is analytically a constant-speed straight line — all derivatives above first order are exactly zero. Supplying them analytically avoids noise amplification from filtering a piecewise-linear waypoint stream.

### 12.2 Dirty Derivatives Only on Actual Velocity

Dirty-derivative filters are retained exclusively for estimating `v̇` and `v̈` (actual acceleration and jerk) — quantities that are not available analytically. Filter time constants τ = 0.05 s (1st derivative) and τ = 0.50 s (2nd derivative) provide a trade-off between phase lag and noise rejection.

### 12.3 SO(3) Projection After Each RK4 Step

Numerical integration of `Ṙ = R Ω̂` allows R to drift off SO(3) over time. SVD projection after every step (not every N steps) is chosen because it is inexpensive (3×3 SVD ≈ 50 flops) and eliminates the source of error at each integration step, keeping `‖R^T R − I‖_F < 10⁻¹²`.

### 12.4 Segment-by-Segment Handoff vs Continuous Integration

The IFDS planner re-runs every `dt_traj = 1 s`. Each new plan may alter the waypoint sequence significantly. A segment-by-segment architecture (one `SE3Track` call per IFDS waypoint segment) was chosen over a single long integration because:
- It naturally respects the IFDS re-plan cadence.
- The controller state (R, Ω, v) transfers continuously — there is no reset.
- The time-budget mechanism (`dt_budget = dt_traj − dtcum`) ensures the UAV spends at most `dt_traj` seconds executing a stale plan before IFDS replans.

---

## 13. References

1. T. Lee, M. Leok, and N. H. McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3)," *Proceedings of the 49th IEEE Conference on Decision and Control*, Atlanta, GA, 2010. [arXiv:1003.2005v4]

2. T. Lee, M. Leok, and N. H. McClamroch, "Stable manifolds of saddle equilibria for pendulum dynamics on S² and SO(3)," *Proceedings of the 50th IEEE Conference on Decision and Control*, Orlando, FL, 2011.

3. K. Tamanakijprasart, "Dynamic Autorouting using Iterative Forward–Backward Dynamic Systems (IFDS)," University Project Report, 2023.

4. J. Thomas, `se3quad` MATLAB reference implementation, 2020. https://github.com/jusThomas/se3quad
