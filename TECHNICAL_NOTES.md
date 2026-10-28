# Technical Notes

Reference for the physical model, the numerical methods, the control design and
the engineering decisions taken along the way. For a short overview see
[README.md](README.md).

---

## 1. Physical model

### 1.1 State vector

The system state is four values forming a single object:

| Index | Symbol | Meaning |
| --- | --- | --- |
| 0 | `x` | cart position on the rail (m) |
| 1 | `x_dot` | cart velocity (m/s) |
| 2 | `theta` | pendulum angle from upright (rad, in `]-pi, +pi]`) |
| 3 | `theta_dot` | angular velocity (rad/s) |

`theta = 0` is the **upright** (unstable) equilibrium, `theta = pi` the
**hanging** (stable) one. This convention matters: `theta = 0` being upright is
what makes the stabilisation controller a plain regulation problem, and it is
assumed by the renderer when it draws the bob.

### 1.2 Equations of motion

`compute_derivatives()` returns `dY/dt`. Two structural properties make
numerical integration possible at all:

- the derivative has the **same shape as the state** (four values),
- it is a pure function of the current state: `dY/dt = f(Y)`.

With the coupling denominator `D(theta) = M + m*sin^2(theta)`:

```
x_ddot     = [ F - b*x_dot + m*L*omega^2*sin(theta) - m*g*sin(theta)*cos(theta)
                  + (c*omega*cos(theta))/L ] / D

theta_ddot = [ g*sin(theta) - x_ddot*cos(theta) - (c*omega)/(m*L) ] / L
```

The term `m*L*omega^2*sin(theta)` is the centripetal coupling from the swinging
bob to the cart; the `c*omega/L` terms are the pivot damper acting on both the
cart and the pendulum.

### 1.3 Default parameters

| Symbol | Value | Meaning |
| --- | --- | --- |
| `M` | 1.0 kg | cart mass |
| `m` | 0.2 kg | pendulum bob mass |
| `L` | 1.0 m | rigid rod length |
| `g` | 9.81 m/s² | gravity |
| `b` | 0.1 N·s/m | cart viscous friction |
| `c` | 0.01 N·m·s/rad | pivot damping |
| `F_max` | 9 N | motor saturation |
| `x_max` | 5.0 m | rail half-length |

---

## 2. Numerical integration

### 2.1 The scheme

`integrate_rk4()` advances the state by one step of duration `dt`. The
derivative is sampled four times, at four different positions on the step, then
combined as a weighted average:

```
k1 = f(Y)
k2 = f(Y + (dt/2)*k1)
k3 = f(Y + (dt/2)*k2)
k4 = f(Y +  dt  *k3)

Y(t+dt) = Y(t) + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
```

The `1-2-2-1` weighting is the signature of the **Simpson quadrature rule**.
Those weights are not a heuristic: they cancel the leading error terms
analytically, which is what makes the method exact to order 4 instead of merely
"better than Euler".

The intuition is a road with curves. Euler looks at the slope once, at the
start, and cuts every corner. RK4 reads the slope at four points and averages
them, so the corners are rounded instead.

### 2.2 Invariants during a call

- `state` is **not written** until the final combination. This is what preserves
  `Y(t)` as the base for all four samples without needing a defensive copy.
- The three intermediate states are always rebuilt from `Y(t)`, never from an
  earlier intermediate state. Rebuilding from an intermediate state would
  silently produce a different (first-order) method.

### 2.3 The index coupling

`StateDerivative` is the state shifted by one step in the "rate" sense:
`deriv[i]` is the rate of change of `state[i]`. The update rule is therefore
uniform across all four components:

```
state[i](t+dt) = state[i](t) + (dt/6)*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i])
```

Using the same index on both sides removes an entire class of bug: `x` can only
advance with a velocity and `x_dot` only with an acceleration, because that is
what the layout of the two structures already encodes.

### 2.4 Post-conditions

After the combination, the state is normalised in two ways.

**Angle wrapping.** `theta = wrap_angle_pi(theta)`. The angle is a periodic
quantity, not a real one: `theta`, `theta + 2pi` and `theta + 4pi` denote the same
physical orientation. Left unwrapped, the stored value grows without bound and
loses floating-point precision over time. The invariant must hold *between* two
steps, so the next step starts from a normalised state.

**Rail bumper.** When `|x|` exceeds `x_max`, `x` is clamped to the nearest stop
and `x_dot` is zeroed. This models an inelastic collision: the cart stops and
kinetic energy is dissipated. The velocity is cleared **only on contact**, never
unconditionally — otherwise the cart would be frozen at every step.

The bumper is a **brake, never an actuator**. Both of its operations can only
remove energy; no term adds any. It cannot help lift the pendulum.

### 2.5 Measured accuracy

Energy drift over 5000 steps with friction and damping disabled:
**1.07e-11 J**, against a tolerance of `1e-6 J`.

Small-angle period, cross-checked against theory:

| Configuration | Measured | Expected |
| --- | --- | --- |
| Free cart, frictionless | 1.8314 s | 1.8313 s (`2*pi*sqrt(M*L/((M+m)*g))`) |
| Blocked cart (`M = 1e9`) | 2.0061 s | 2.0061 s (`2*pi*sqrt(L/g)`) |

The free-cart period is 9 % shorter than the isolated pendulum because the cart
recoils and changes the effective inertia. The blocked-cart case recovering the
isolated value exactly validates the whole chain: physics, integrator and
measurement.

---

## 3. Control

### 3.1 Hybrid structure

Two control laws are used, each valid only in its own regime.

| Mode | Law | Valid for |
| --- | --- | --- |
| `CTRL_OFF` | motor disengaged | free passive dynamics |
| `CTRL_SWING_UP` | energy pumping | `\|theta\| > threshold` |
| `CTRL_STABILIZATION` | linear state feedback | `\|theta\| <= threshold` |

**Why a switch is unavoidable.** The stabilisation law is a linearisation about
`theta = 0`, where `sin(theta) ≈ theta`. Its error grows as `theta^3/6`:

| `theta` (rad) | error | usable |
| --- | --- | --- |
| 0.1 | 0.02 % | yes |
| 0.35 | 2 % | acceptable |
| 1.0 | 17 % | already poor |
| pi | enormous | meaningless |

The swing-up law fails at the opposite end. Near `theta = 0` the factor
`cos(theta)` tends to 1, so the law reduces to `sign(theta_dot)`. At the
equilibrium `theta_dot` is nearly zero, and **the sign of a nearly-zero quantity
is not information but noise** — the result is a full-amplitude force in a
direction determined by the last bit of the floating-point representation.

Neither law can substitute for the other. The two curves of validity are
complementary, which is what makes a state machine necessary here.

**Why the threshold is 0.35 rad.** It sits where the linearisation error stays
near 2 %, and it creates a dead band. Without one, the mode would chatter across
the boundary every step, injecting a discontinuity into the command at 1 kHz.

`CTRL_OFF` is **latched**: it is preserved until the user explicitly cycles the
mode, otherwise the controller would re-enable itself one step after being
switched off.

### 3.2 Swing-up

```
F = k_E * E_rel * sign(theta_dot * cos(theta))
```

`E_rel` comes from `compute_pendulum_energy()`, computed **relative to the
upright rest position**:

```
E_rel = 0.5*m*L^2*theta_dot^2 + m*g*L*(cos(theta) - 1)
```

The `- 1` fixes the zero of the scale on the target equilibrium, so the target
energy is **0 by construction** and the `(E_rel - E_target)` term collapses to
`E_rel`. No extra state is needed to store a target that is identically zero.

`E_rel` is negative while the pendulum hangs (about `-2*m*g*L`) and crosses zero
exactly at the equilibrium being sought, so it reads as an "energy still
missing" measure.

The sign factor pushes the pendulum along its current motion to pump energy
upward on each swing.

A dedicated `sign_with_zero()` helper is used because the natural-looking
`v / fabs(v)` idiom divides by zero when `v` is exactly 0, which does occur.

### 3.3 Stabilisation

```
F = K_x*(x - x_ref) + K_vx*x_dot + K_theta*theta + K_vtheta*theta_dot
      + K_i*error_integral
```

| Gain | Value |
| --- | --- |
| `K_x` | 5.00 |
| `K_vx` | 11.25 |
| `K_theta` | 78.00 |
| `K_vtheta` | 45.75 |
| `K_i` | 0.00 |

### 3.4 Why the position gains are positive

`K_x` and `K_vx` are **positive**, opposite to the usual "pull the cart back to
`x_ref`" intuition. This is not a sign error.

The cart's freedom of movement is entirely consumed by keeping the pendulum up.
A restoring term on the cart fights the pendulum regulation and leaves the cart
parked at a rail end without recentring. Closing the loop around the pendulum
instead lets the cart recentre as a consequence.

A constrained variant, with `K_x < 0` and `K_vx < 0` enforced, was optimised the
same way and compared:

| Variant | Cost | Convergence |
| --- | --- | --- |
| suggested by the specification (`K_x = -15`, `K_vx = -20`) | 2427 | 0/16 |
| **positive, adopted** | **0.91** | **16/16** |
| constrained to negative | 330 | 0/16 |

The constrained optimum pushes `K_theta` to about 8100, which saturates the
motor continuously and still never recentres the cart. The positive solution is
roughly 350x better.

### 3.5 Integral action

Disabled (`K_i = 0`). On this plant the integral term only introduces windup: at
`|K_i| >= 20` the pendulum topples. A working integral action would require an
anti-windup scheme (integrator clamping), which is a separate piece of work.

### 3.6 How the gains were obtained

Direct optimisation of the closed loop against the **non-linear** plant, rather
than a linear-quadratic design. The usual route — linearise, then solve the
algebraic Riccati equation — diverges here, because the inverted equilibrium is
unstable (pole at about `+3.46 rad/s`) and the fixed-point iteration used to
solve the Riccati equation does not converge for an unstable system.

A coordinate descent was run instead over a cost measured across 108 initial
conditions, then validated on scenarios never used during the search:

| Check | Result |
| --- | --- |
| Stabilisation, 108 initial conditions | 108/108 |
| Disturbance rejection (up to 15 N for 50 ms) | 16/16 |
| Swing-up, 6 starting angles | 6/6, mean lock time 2.13 s |
| Peak motor saturation | 0.1 % of the run |
| Worst final state | `\|theta\| = 5e-4 rad`, `\|x\| = 3e-2 m` |

The swing-up gain was chosen from the middle of its usable range: `k_E` between
about 35 and 80 works across all tested thresholds, and 45 sits in the centre.

### 3.7 Actuator and rail sizing

The rail and the motor are dimensioned together, and the pair interacts strongly.
The swing-up law is **bang-bang**: it commands full torque whenever the pendulum
is below the target energy. The command reaches `k_E * 2*m*g*L = 176 N` at the
hanging position, so it saturates at every step. Consequently the cart accelerates
at full tilt until the control reverses, and on a short rail it arrives at the
bumper at high speed.

An energy budget over the swing-up, accounting for motor work against bumper
losses, measured across a sweep of rail lengths and motor torques:

| Rail | `F_max` | First lock | Permanent lock | Bumper loss | Motor work |
| --- | --- | --- | --- | --- | --- |
| 2.4 m | 20 N | 1.06 s | 3.52 s | 83 % | 45.1 J |
| 3.5 m | 20 N | 1.06 s | 1.06 s | 67 % | 20.1 J |
| **5.0 m** | **9 N** | **2.11 s** | **2.11 s** | **20 %** | **8.7 J** |
| 5.0 m | 5 N | 3.41 s | 3.41 s | 4 % | 7.1 J |
| 2.4 m | 3 N | 7.18 s | 7.18 s | 15 % | 7.2 J |

Three conclusions:

- **Lengthening the rail and weakening the motor are complementary.** Together
  they cut the wasted energy from 83 % to 20 % while also *reducing* the time to
  a stable lock.
- **Weakening the motor helps more than lengthening the rail.** A slower
  acceleration gives the control time to reverse before contact, so the impact
  speeds collapse (measured first impact: 7.46 m/s down to 1.40 m/s).
- **There is a speed/efficiency trade-off.** The fastest locks happen exactly when
  the system wastes the most energy, because a slam is a fast way to move.

The first configuration in the table also shows a defect worth naming: it reaches
`|theta| <= threshold` at 1.06 s but **falls back into swing-up twice** before
staying locked at 3.52 s. The 34.5 J impact at 0.53 s is the cause. Removing the
collisions removes the chattering between the two control laws as a side effect.

---

## 4. Interface

### 4.1 Bitmap font

On-canvas text uses a 5x7 bitmap font embedded directly in `renderer.c`,
covering ASCII `0x20` to `0x5A`. This avoids a dependency on `SDL2_ttf` (which
may not be installed) and avoids shipping a font file.

Consecutive lit pixels within a column are merged into a single rectangle, which
cuts the number of SDL draw calls by roughly 3x compared to per-pixel
rendering.

### 4.2 Control mode banner

A panel in the top-left shows the active mode, colour-coded so the state is
readable without reading the text:

| Mode | Label | Colour |
| --- | --- | --- |
| `CTRL_OFF` | `FREE MODE` | red |
| `CTRL_SWING_UP` | `SWING-UP` | amber |
| `CTRL_STABILIZATION` | `VERTICAL BALANCE` | green |

A telemetry row (energy, `|theta|`, motor saturation) sits below the mode name
when the motor is driven. It is omitted in `FREE MODE`, where no control law is
running and the values carry no operational meaning.

The mode label is produced by a single function used by both the banner and the
window title, so the two can never disagree. The switch over `ControlMode` is
exhaustive, so adding a mode without a label is a compile error rather than a
silent fallback.

### 4.3 The circular angle problem

This is the single subtlest issue in the whole project and it recurs in three
places.

`theta` is stored **modulo 2*pi**, in `]-pi, +pi]`. When the pendulum crosses the
hanging position, the physical orientation is continuous but the stored number
jumps:

```
physical: ... 3.10  ->  3.1416  ->  3.18 ...     (continuous motion)
stored:      3.10  ->  3.1416  ->  -3.10 ...    (jump of -2*pi)
```

`+3.1416` and `-3.1416` denote the same orientation. **Any logic that compares an
angle against `+pi` or `-pi` as if the angle were a real number is broken**, since
the variable can cross that boundary without the value ever reaching it.

This affected:

1. the periodic period test, whose crossing detection could never trigger;
2. the angle strip chart, which drew a vertical spike every half period;
3. any reasoning about "above" or "below" the hanging position.

### 4.4 Phase unwrapping for the angle plot

The information lost by the modulo cannot be recovered from a single sample — the
two candidates are indistinguishable. It **can** be recovered incrementally,
because between two consecutive samples the angle cannot have moved much.

```
delta = theta_stored - theta_unwrapped_previous
if delta >  +pi : delta -= 2*pi
if delta <  -pi : delta += 2*pi
theta_unwrapped = theta_unwrapped_previous + delta
```

The correction only fires when `|delta| > pi`, which is detectable: a genuine
motion of more than `pi` between two samples would be physically absurd here.
The peak angular velocity measured over the whole simulation is 6.28 rad/s
(reached during swing-up); at 100 Hz that is 0.063 rad between samples, against a
threshold of 3.14 rad — a **safety margin of 50x**.

`state.theta` itself is never modified. The roll-up exists only for display.

If the pendulum completes a full turn, the unwrapped value legitimately grows by
`2*pi`: that is a real property of the rotation, not a runaway. The vertical
axis is therefore centred on the **mean of the visible window**, and its
half-height is capped at `4*pi`, which keeps the oscillation readable instead of
letting it collapse onto an edge.

### 4.5 Strip charts

Three live plots run along the bottom of the window.

**Sampling.** They are fed at a fixed 100 Hz from the **physics** loop, not from
the render loop, so the time axis stays uniform even if the frame rate
fluctuates. A ring buffer of 1400 samples gives a 14-second window. The
accumulator is floating point: an integer step counter written as
`(int)(PLOT_RATE_HZ * dt)` truncates to `0` and silently feeds at the full 1 kHz
physics rate instead.

**theta(t).** The unwrapped angle, recentred on the window mean as described
above.

**Phase portrait — `theta_dot` versus `theta`.** The standard figure for a
one-degree-of-freedom oscillator: a closed ellipse indicates a conservative
system, an inward spiral indicates dissipation, a single point indicates
equilibrium. Both axes use the **same pixel size per unit**, otherwise the orbits
are sheared and a circle no longer looks like a circle. The trajectory is drawn
as dots fading with age rather than a polyline, because consecutive samples are
adjacent in time while the orbit is closed — a line would smear the loops.

**Energy versus time.** The slope is the mechanical power. Free mode gives a
monotonic decay; swing-up shows the motor charging the system; in stabilisation
the curve hovers around a constant value as the controller pumps and dumps
energy.

### 4.6 Layout

The canvas is 1280x720. The rail sits at `h/2 - 30` with a scale of 120 px/m, so
the animation occupies the upper part and the chart strip the lower part. The
renderer is written entirely with SDL primitives, so no texture assets are
needed.

**Sizing constraint.** The rail is drawn as `origin_x ± x_max * PIXELS_PER_METER`,
so the window width bounds the rail: `x_max * PIXELS_PER_METER <= 600`. Lengthening
the rail therefore requires reducing the scale, or the rail is drawn off-screen.
The physics and the bumper are unaffected — they read `x_max` from the parameters
and adapt on their own.

### 4.7 Axis label formatting

`%.2g` renders `100` as `1e+02`, which does not fit a panel and reads badly in
the bitmap font (which has no legible lowercase `e`). Precision is therefore
chosen from the magnitude of the value instead, and scientific notation is only
used outside the useful range.

---

## 5. Tests

`make tests` builds a standalone CLI suite with no SDL dependency. Four tests:

| Test | Verifies |
| --- | --- |
| `test_wrap_angle` | angle normalisation into `]-pi, +pi]` |
| `test_equilibrium_derivatives` | zero acceleration at both equilibria |
| `test_energy_conservation_rk4` | energy constant within `1e-6 J` over 5000 steps |
| `test_small_angle_period` | quarter period within 1 % against theory |

Tests 3 and 4 are **invariant tests**: they check a physical property rather than
comparing against a precomputed reference. The benefit is that no correct answer
needs to be known in advance — only the physics. An invariant test detects whole
families of defects instead of a single set of expected values.

Test 3 disables friction and damping so that a conservative system results and
the energy drift isolates the integrator quality. It also sets `x_max = 50` so
the rail bumpers stay out of the way and only the integration scheme is measured.

Test 4 disables the rail effect with `M = 1e9` (effectively a blocked cart) and
compares against the closed-form period. It tracks the signed offset
`wrap_angle_pi(theta - pi)` rather than `theta` itself, which places the hanging
position at 0 in the middle of the interval instead of at the wrap boundary — see
section 4.3.

---

## 6. Defects found and fixed

Recorded because every one of them is silent: none crashes, none warns, none
produces `NaN`. They were found by reasoning about the physics and by checking
the rendered output, not by the compiler.

| # | Defect | Symptom |
| --- | --- | --- |
| 1 | Rail clamp guard was `!(x <= x_max)` with `fabs` applied to `x_max` instead of `x` | every negative `x` satisfies `x <= x_max`, so the clamp never fired below centre; the cart ran to `x = -5.7 m` with `x_max = 2.4` |
| 2 | `compute_motor_command` switch used `case 3` on an enum whose maximum is 2, had no `default`, and read `F_max` from `Controller` instead of `SystemParams` | wrong mode reached, undefined return, compile error |
| 3 | Swing-up law used `theta_dot / fabs(theta_dot)` | division by zero whenever `theta_dot` is exactly 0 |
| 4 | Swing-up law called `compute_total_energy` instead of `compute_pendulum_energy` | cart kinetic energy and a non-relative potential in an energy balance that must be relative |
| 5 | Period test detected the crossing as `theta >= M_PI` | unwrapping makes `theta >= M_PI` unreachable, so the test could never pass |
| 6 | Plot decimation written as `(int)(PLOT_RATE_HZ * dt)` | truncates `100 * 0.001` to `0`; the buffer filled at 1 kHz and the 14 s window showed 1.4 s |
| 7 | Axis labels formatted with `%.2g` | `100` rendered as `1e+02`, which collided with the panel title and read as `1 +02` |

### A note on test coverage

Shifting the meaning of `theta` by `-pi` inside the integrator would break the
renderer, the controller and the telemetry, and **all four tests would still
pass**: the shift does not violate normalisation, does not move the equilibria
(`sin(0) = sin(pi) = 0`), and does not break energy conservation (the shifted
system conserves its own energy).

This is the general lesson: a passing test proves the property it tests, not
conformance to the contract. `theta` is a shared data contract between four
modules, and only the module that defines it may decide its origin.

---

## 7. Timing model

The renderer runs at roughly 60 Hz. Real elapsed wall-clock time is measured
with the SDL performance counter, accumulated, and drained in fixed 1 ms physics
steps, so the simulation rate is independent of the frame rate.

The accumulator is clamped to 100 ms to prevent a spiral of death after a stall
(a long pause would otherwise queue an unbounded number of catch-up steps).

Saturation is applied in `main.c` immediately before integration, and again
inside `compute_motor_command`. The duplication is harmless — saturation is
idempotent — and both layers are independently reasonable.