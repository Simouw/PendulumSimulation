# Inverted Pendulum on a Cart

Real-time simulation of a cart-pole system with an automatic balancing
controller, an SDL2 interface and live graphs.

---

## What it does

A cart slides on a horizontal rail with a pendulum mounted on it. The pendulum
is **inverted**, so it wants to fall. A controller drives the cart to keep it
upright.

You can watch it swing up from the hanging position and lock to vertical in about
two seconds, watch it decay freely with no motor at all, or push it around by hand
with the arrow keys and see the graphs react.

---

## How it works

The project is split into three responsibilities that never step on each other:

- **The physics** decides where the system is one millisecond from now, given
  where it is now. It never looks at the controller.
- **The controller** looks at the current state and decides how hard to push the
  cart. It never touches the physics.
- **The interface** draws whatever state the physics is in. It never influences
  either of the other two.

The simulation runs at a fixed 1000 steps per second regardless of the frame
rate: the renderer measures how much real time has passed and drains the surplus
in fixed 1 ms steps. So the physics stays identical whether the display manages
60 fps or drops to 20.

Two different control strategies are used depending on where the pendulum
currently is:

| Situation | Strategy |
| --- | --- |
| Motor off | Nothing. Free, passive motion. |
| Far from vertical | **Swing-up** — the cart pumps energy into the pendulum until it stands up. |
| Close to vertical | **Stabilisation** — a feedback law holds it exactly at vertical and brings the cart back to the centre of the rail. |

The switch is on the pendulum angle and costs nothing to evaluate. It matters
because each strategy only works in its own regime: the fine-tuning law assumes
the pendulum is nearly vertical and misbehaves badly otherwise, while the
swing-up law becomes unusable noise near vertical. Neither can replace the other.

Three live graphs run along the bottom of the window: the angle over time, a
phase portrait, and the mechanical energy.

---

## Build and run

```bash
make          # build bin/pendulum_sim
make tests    # run the unit test suite
make run      # build and launch the simulation
make clean    # remove build artefacts
```

Requirements: a C11 compiler, SDL2 and `make`.

```bash
# Debian / Ubuntu
sudo apt install build-essential libsdl2-dev
```

The project compiles with `-Wall -Wextra -Werror -std=c11 -O2`: any warning
fails the build.

---

## Controls

| Key | Action |
| --- | --- |
| `SPACE` | Reset with the pendulum tilted near vertical |
| `B` | Reset with the pendulum hanging down |
| `C` or `TAB` | Switch mode: off → swing-up → balance |
| `LEFT` / `RIGHT` | Push the cart by hand (±12 N) |
| `ESC` or `Q` | Quit |

---

## Worth trying

- Press `B` then `C`. Watch the phase portrait spiral collapse to a point — that
  is the controller taking over.
- Press `C` again to switch back to free mode and let it fall. The energy graph
  now decays instead of staying flat.
- Hold an arrow key in balance mode. The energy graph starts oscillating: that is
  the motor working to reject the disturbance.
- Compare the graphs in free mode and in balance mode. In free mode the amplitude
  shrinks; in balance mode the trajectory converges on the origin.

---

## Configuration

Physical parameters and timing live in [`include/config.h`](include/config.h).
The defaults are a 1 kg cart, a 0.2 kg bob, a 1 m rod, 9 N of motor torque and a
10 m rail. The integrator runs at 1 kHz, the display at 60 Hz.

Controller gains live in `init_controller()` in
[`src/controller.c`](src/controller.c).

---

## Project layout

```
include/   headers: physics, integrator, controller, renderer, config
src/       physics.c  integrator.c  controller.c  renderer.c  main.c
tests/     test_main.c — standalone test suite, no SDL dependency
```

`make tests` builds and runs four tests covering angle normalisation, the
equilibrium derivatives, energy conservation under the integrator, and the
oscillation period against theory.

---

## Further reading

The physics model, the integration scheme, the control design and the reasoning
behind the various engineering decisions are documented in
[TECHNICAL_NOTES.md](TECHNICAL_NOTES.md).

---

## Credits

Physics, dynamics and control design by the project author. Interface,
visualisation and test suite produced with AI assistance.