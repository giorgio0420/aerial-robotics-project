# Aerial manipulator — tube-based LPV-MPC

![The hexarotor tracking the nominal trajectory in Gazebo while the arm steps its joints](media/flight.gif)

A hexarotor carrying a three-link arm has a problem a plain drone does not: every
time the arm moves, it pushes back on the vehicle. Swing the shoulder and the
aircraft rolls; accelerate the wrist and the attitude loop sees a torque it never
asked for. The usual fix is to estimate that reaction and cancel it after the
fact, which works while the arm is slow and falls apart when it is not.

This project reimplements, in ROS 2 and Gazebo, a scheme that treats the reaction
differently: it is written out in terms of the vehicle's own states, folded into a
linear parameter-varying (LPV) model, and handed to a robust MPC that plans with
it instead of reacting to it. One MPC flies the vehicle's rotation, a second one
drives the arm, and both are compared against a reaction-cancelling baseline.

It is an independent reimplementation of the method described in

> A. Eskandarpour, M. Soltanshah, K. Gupta, M. Mehrandezh,
> *Decoupled Dynamic Modeling by Decomposing the Cross-Coupled Dynamics and
> Tube-Based LPV-MPC Control Scheme for Aerial Manipulation*,
> IEEE Transactions on Aerospace and Electronic Systems, 61(5), 2025.
> [doi:10.1109/TAES.2025.3576083](https://doi.org/10.1109/TAES.2025.3576083)

written for the Aerial Robotics course at Sapienza. The paper itself is not
included here; the theory below is a summary in my own words of what the code
does, and everything that departs from the original is stated as such. This
repository is not affiliated with the authors.

---

## 📖 Theory

### The coupling problem

The vehicle is a rigid body with position $\xi$, attitude $\eta = (\phi, \theta, \psi)$
and body rate $\omega = (p, q, r)$. The arm hangs below it with joint angles
$\Theta \in \mathbb{R}^3$. Newton–Euler for the vehicle alone would read

```math
m\ddot{\xi} = R(\eta)\, f_z e_3 - m g e_3 + f_{rea}, \qquad
I\dot{\omega} = -\omega \times I\omega + \tau + \tau_{rea}
```

and everything interesting sits in $f_{rea}$ and $\tau_{rea}$, the force and torque
the arm exerts on the vehicle at its mount. They depend on the arm's configuration,
its velocities and accelerations — and on the vehicle's own motion, because the arm
is riding on it.

### Writing the reaction in the vehicle's states

Run the recursive Newton–Euler equations of the arm outward from the vehicle and
back inward, and look at the torque that arrives at the base. It is affine in the
vehicle's angular acceleration, exactly quadratic in its angular rate and affine in
the thrust. That structure is what the method exploits: the reaction torque can be
factored as

```math
\tau_{rea} = M_d(\Theta)\,\dot{\omega}
+ M_c(\Theta)\begin{bmatrix} pq \\ qr \\ pr \end{bmatrix}
+ M_s(\Theta)\begin{bmatrix} p^2 \\ q^2 \\ r^2 \end{bmatrix}
+ M_l(\Theta,\dot\Theta)\,\omega + \bar{\tau}
```

and the reaction force as $f_{rea} = M_f  f_z + \bar{f}$. The matrices change with
the arm's configuration; $\bar\tau$ and $\bar f$ collect whatever does not depend on
the vehicle's rotation or thrust.

Written this way the coupling stops being a disturbance. $M_d$ moves to the left-hand
side and becomes extra inertia, $I - M_d$; the quadratic terms become a state matrix
once one factor of the rate is treated as a scheduling parameter; and only the
residual $\bar\tau$ is left on the input side.

The code gets the matrices in closed form. `derive_dynamics.py` runs the project's own
Newton–Euler recursion symbolically with SymPy and writes `generated_dynamics.py`;
the closed form agrees with the numerical recursion to about $10^{-15}$.

### LPV models

**Rotation.** With $\rho = \omega$ as the scheduling signal,

```math
\dot\omega = (I - M_d)^{-1}\Big[\big(M_c^{tot}\,\mathrm{diag}(q, r, p) + M_s^{tot}\,\mathrm{diag}(p, q, r) + M_l\big)\,\omega + (\tau + \bar\tau)\Big]
```

where $M_c^{tot}$ and $M_s^{tot}$ include the vehicle's own gyroscopic term. The state
is the body rate $[p, q, r]$, the input is $\tau + \bar\tau$.

**Arm.** The manipulator's equation $M(\Theta)\ddot\Theta + C \dot\Theta + G(\Theta) = \tau_{arm}$
is put in the same shape by splitting gravity into a part linear in $\Theta$ and a
remainder, $G = Q \Theta - T$. With state $[\Theta, \dot\Theta]$ and input
$\tau_{arm} + T$:

```math
\frac{d}{dt}\begin{bmatrix}\Theta\\ \dot\Theta\end{bmatrix} =
\begin{bmatrix} 0 & I \\ -M^{-1}Q & -M^{-1}C \end{bmatrix}
\begin{bmatrix}\Theta\\ \dot\Theta\end{bmatrix} +
\begin{bmatrix} 0 \\ M^{-1}\end{bmatrix}(\tau_{arm} + T)
```

Both models are rebuilt at every control step from the current measurement and
discretised with a zero-order hold.

### Tube-based MPC

An LPV model is exact at the moment it is built and slightly wrong one step later,
because the parameters keep moving. The tube construction turns that error into a
bounded disturbance and plans around it:

1. A **nominal** system $\bar x_{k+1} = A\bar x_k + B\bar u_k$ is optimised by an ordinary
   QP over a short horizon.
2. The **real** input adds feedback on the gap between the two,
   $u = \bar u + K(x - \bar x)$, with $K$ the LQR gain of the current LPV model.
3. The gap $e = x - \bar x$ stays inside a set that grows along the horizon by how much
   the parameters can drift per step. The QP's state and input limits are
   **tightened** by exactly that set (a Minkowski difference), so whatever the
   feedback adds on top of the nominal plan still fits inside the real limits.

| | what it does |
|---|---|
| $\bar u$ | the optimiser's plan, computed on the nominal model |
| $K(x - \bar x)$ | pulls the real state back toward the plan |
| tightened constraints | leave room for that correction inside the actuator limits |
| scheduled $K$ | re-solved each step, so the tube stays contractive as the arm folds |

### The control cascade

```
position reference ──► PD ──► force demand ──► thrust + attitude reference
                                                       │
                                  geometric attitude error → body-rate reference
                                                       │
                                     tube LPV-MPC (rotation) ──► vehicle torques
joint reference ─────────────────── tube LPV-MPC (arm) ─────────► joint torques
```

The arm's reaction enters in two places: $\bar f$ is subtracted from the force demand
before thrust and attitude are computed, and the full decomposition sits inside the
rotational LPV model.

### Baseline: ERTF

The comparison scheme, *Estimating Reaction Torque and Force*, computes the reaction
with Newton–Euler and cancels it as feedforward, with PID closing the attitude and
joint loops. Its weak point is structural: the estimate needs the vehicle's angular
acceleration, which is only known after the torque has been chosen, so it always
uses last step's value.

### What differs from the paper

- **Position loop.** A PD with critical damping replaces the translational MPC. That
  stage is a plain constrained linear MPC and is not one of the paper's contributions;
  it is kept in the code (`UAM_TRANS_MPC=1`) but not on the default path.
- **Attitude reference.** Roll and pitch are taken from the standard
  body-z-axis construction, symmetric in the two axes, and the attitude error is the
  geometric one on SO(3), which does not degenerate at large angles.
- **Discretisation.** Zero-order hold instead of forward Euler. The arm's mass matrix
  is badly conditioned and forward Euler is unstable at this sample time.
- **Control rate.** 20 Hz instead of the paper's rate: that is what a Python/OSQP
  controller holds against Gazebo on a laptop.
- **Terminal set.** The robust invariant terminal set is computed and verified, but
  is off by default (`UAM_TERMINAL_SET=1`): with a four-step horizon it
  over-constrains the QP. The flying configuration uses a Riccati terminal cost.

---

## 📊 Results

Tracking error in the Gazebo simulation, two runs per case, start-up transient
excluded, as a percentage of the reference amplitude:

| scenario | scheme | position error | joint error |
|---|---|---|---|
| `nominal` | **PD + 2 tube MPC** | **4.9 %** | **4.8 %** |
| `nominal` | PD + ERTF | 14.1 % | 149 % |
| `fast` | **PD + 2 tube MPC** | **9.2 %** | **4.3 %** |
| `fast` | PD + ERTF | 13.0 % | 105 % |

The joint column is the clean comparison: with the reaction handled inside the model
the arm tracks within 5 %, while the baseline's error is larger than the reference
itself. The position column shares the same outer loop across both schemes, so the
gap there is smaller and says less.

---

## 🚀 Running it

Tested on Ubuntu 22.04 (WSL2 works) with ROS 2 Humble and Gazebo Fortress.

```bash
# dependencies, once
sudo apt install -y ros-humble-desktop ros-humble-ros-gz \
  ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-xacro python3-colcon-common-extensions python3-pip

git clone https://github.com/giorgio0420/aerial-robotics-project ~/aerial-robotics-project
cd ~/aerial-robotics-project
pip install -r requirements.txt
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# fly one scenario: timeout in seconds, arm enabled
UAM_SCENARIO=nominal bash phase3.sh 95 true
```

The run writes `/tmp/trace.csv`. To read it:

```bash
python3 tools/iae.py run=/tmp/trace.csv                  # tracking-error table
python3 tools/report.py run=/tmp/trace.csv --out report  # plots
```

| variable | default | effect |
|---|---|---|
| `UAM_SCENARIO` | `nominal` | `hover`, `nominal` or `fast` |
| `UAM_USE_ERTF` | `0` | `1` flies the ERTF baseline instead of the tube MPCs |
| `UAM_TRANS_MPC` | `0` | `1` replaces the position PD with the translational MPC |
| `UAM_GUI` | `1` | `0` runs Gazebo headless, much cheaper on CPU |
| `UAM_TERMINAL_SET` | `0` | `1` imposes the robust terminal set |

Without a GPU, `phase3.sh` already forces software rendering. The GUI alone costs
several times the CPU of the control loop, so use `UAM_GUI=0` for measurements.

## Layout

```
phase3.sh                          one run: Gazebo, spawn, controller, release
src/hexacopter_sim/                XACRO model, launch file, ROS–Gazebo bridge
src/hexacopter_control/            Gazebo plugin: rotor speeds -> forces and torques
src/uam_control/hover_node.py      flight node: cascade, allocation, trace
src/uam_control/uam_control/       dynamics, decomposition, MPCs, ERTF, trajectories
src/uam_control/derive_dynamics.py regenerates generated_dynamics.py with SymPy
tools/                             iae.py, report.py — read a trace.csv
```
