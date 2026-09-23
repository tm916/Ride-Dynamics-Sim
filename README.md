# 2-DOF Quarter-Car Ride Model

A Python model of one corner of a Formula Student car. It simulates how the body (sprung mass) and the wheel (unsprung mass) respond to a sinusoidal road input. A parameter study then measures how spring rate and damping change the body amplitude at a fixed road frequency.

Parameters are loosely based on the Cambridge University Full Blue Racing car.


## Model

Two masses in series:

- The sprung mass `m_s` (body) sits on the suspension spring `k_s` and damper `b_s`.
- The unsprung mass `m_u` (wheel) sits on the tyre spring `k_u` and tyre damping `b_u`.
- The road input `z_r` acts under the tyre.

Equations of motion:

$$m_s \ddot{z}_s = k_s(z_u - z_s) + b_s(\dot{z}_u - \dot{z}_s)$$

$$m_u \ddot{z}_u = k_s(z_s - z_u) + b_s(\dot{z}_s - \dot{z}_u) + k_u(z_r - z_u) + b_u(\dot{z}_r - \dot{z}_u)$$

$$z_r = A \sin(\omega t)$$

Assumptions:

- All springs and dampers are linear.
- Displacements are measured from static equilibrium, so gravity does not appear.
- The tyre stays in contact with the road.
- There is no aerodynamic downforce.
- The mass is split equally over the four corners.

### Parameters

| Symbol | Value | Description | Source |
|---|---|---|---|
| `m_s` | 68.8 kg | Sprung mass per corner | 320 kg car + driver ÷ 4, minus `m_u` |
| `m_u` | 11.2 kg | Unsprung mass per corner | Estimate |
| `k_s` | 25,000 N/m | Suspension wheel rate (baseline) | Chosen |
| `b_s` | 1,500 Ns/m | Damper wheel rate (baseline) | Chosen |
| `k_u` | 100,000 N/m | Tyre vertical stiffness | Estimate |
| `b_u` | 150 Ns/m | Tyre damping | Estimate |
| `A` | 0.02 m | Road amplitude | Chosen |
| `ω` | 31.4 rad/s (5 Hz) | Road frequency | Chosen |

`k_s` and `b_s` are wheel rates. The actual spring and damper rates are the wheel rates divided by MR², where MR is the motion ratio.

### Expected natural frequencies

The tyre acts in series with the suspension spring. As a result, the body sees a softer ride rate:

$$k_{ride} = \frac{k_s k_u}{k_s + k_u} = 20{,}000 \text{ N/m}$$

The eigenvalues of the 2-DOF system give:

| Mode | rad/s | Hz |
|---|---|---|
| Body (heave) | 17.0 | 2.7 |
| Wheel hop | 106 | 16.9 |

The baseline damping ratio of the body mode is ζ = b_s / (2√(k_ride · m_s)) ≈ 0.64.

The road frequency (5 Hz) is between the two modes.

## Numerical method

The model uses fixed-step velocity Verlet, with a velocity predictor for the damper forces.

Standard velocity Verlet needs the acceleration at the next step. Here, the damper forces depend on velocity. So the next acceleration depends on the next velocity, which is not yet known. Each step therefore does four things:

1. It updates the positions with the current acceleration.
2. It predicts the next velocities with an Euler step.
3. It calculates the next acceleration from the new positions and the predicted velocities.
4. It corrects the velocities with the average of the current and next accelerations.

The time step is dt = 2⁻¹⁰ s ≈ 0.98 ms. This is far below the stability limit of about 2/ω_max ≈ 19 ms.

The script stores the state at the start of each step, so index `i` matches `time_array[i]`.

The earlier version, `legacy/quarter_car_euler.py`, uses semi-implicit (Euler–Cromer) integration. That method is first order. It also uses an older sprung mass of 95.2 kg. It is kept for reference only.

## Parameter study

The script sweeps `k_s` and `b_s` on a 6 × 6 grid at a fixed road frequency of 5 Hz.

| Parameter | Range | Range check |
|---|---|---|
| `k_s` | 15,000–40,000 N/m | Ride frequency ≈ 2.2–3.2 Hz |
| `b_s` | 500–3,000 Ns/m | ζ ≈ 0.2–1.3 at the baseline spring |

A typical ride damping target is ζ ≈ 0.5–0.8, so the damping range brackets it.

**Steady-state amplitude.** The simulation starts from rest. So the early response includes a transient at the body mode. The script measures the body amplitude over the last 5 road periods only, as half the peak-to-peak value. At the lowest damping (500 Ns/m), the transient decays in about 1.5 s. A 5 s run is therefore long enough.

**Output.** `amplitude_study.csv`, with the columns `k_s`, `b_s` and `amp_body` (m).

## Baseline result

At `k_s` = 25,000 N/m and `b_s` = 1,500 Ns/m, the steady-state body amplitude is 18.5 mm for a 20 mm road input.

## MATLAB analysis (in progress)

The MATLAB script loads `amplitude_study.csv` and plots the body amplitude as a surface over `k_s` and `b_s`.

## Simulink cross-check (in progress)

A Simulink model of the same equations acts as the reference. It uses ode45 at a tight tolerance. The Python and Simulink results are compared point by point at the same time steps.

## How to run

Install Python 3 with NumPy and Matplotlib:

```
pip install numpy matplotlib
python quarter_car_verlet.py
```

The script shows the baseline velocity and displacement plots. Then it runs the parameter study and writes `amplitude_study.csv`.

## Files

| File | Contents |
|---|---|
| `Verlet_Integrator.py` | Model, baseline plots and parameter study |
| `Euler_Integrator_old.py` | First version, Euler–Cromer integration |


## Planned

- A transmissibility sweep over road frequency.
- Aerodynamic downforce.

