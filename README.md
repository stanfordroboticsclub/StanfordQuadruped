# Stanford Quadruped Gym 

This repo is adapted from the original Standford Quadruped repo at https://github.com/stanfordroboticsclub/StanfordQuadruped and we've added an additional simulator and gym-compatible environments
 
![Pupper CC Max Morse](https://live.staticflickr.com/65535/49614690753_78edca83bc_4k.jpg)


## Installation

We assume you got an environment where `pip` and `python` point to the Python 3 binaries. This repo was tested on Python 3.6.

```bash
git clone https://github.com/fgolemo/StanfordQuadruped.git
cd StanfordQuadruped
pip install -e . 
```

## Getting Started

The new simulator lives in `stanford_quad/sim/simulator2.py` in the `PupperSim2` class. You can run the simulator in dev mode by running the script

    python scripts/07-debug-new-simulator.py
    
## Gym Environments

There are currently 2 environments. Both come in 2 variants: `Headless` and `Graphical`. `Headless` is meant for training and launches the PyBullet sim without any GUI, `Graphical` is meant for local debugging, offers a GUI to inspect the robot and is significantly slower.

You can try out one of the walking environments, by running:

    python scripts/08-run-walker-gym-env.py

### Walking

In both environments, the observations are the same:

**Observation space**:
- 12 leg joints in the order
  - front right hip
  - front right upper leg
  - front right lower leg
  - front left hip/upper/lower leg
  - back right hip/upper/lower leg
  - back left hip/upper/lower leg
- 3 body orientation in euler angles
- 2 linear velocity (only along the plane, we don't care about z velocity

The joints are normalized to be in [-1,1] but the orientation and velocity can be arbitrarily large.

The **action space** in both environments is also 12-dimensional (corresponding to the same 12 joints as above in that order) and also normalized in [-1,1] but the effects are different between both environments (see below).

In both environments, the goal is to walk/run as fast as possible straight forward. This means the **reward** is calculated as relative increase of x position with respect to the previous timestep (minus a small penalty term for high action values, same as in HalfCheetah-v2)

#### Pupper-Walk-Absolute-[Headless|Graphical]-v0

In this environment, you have full absolute control over all joints and their resting position is set to 0 (which looks unnatural - the robot is standing fully straight, legs extended). Any action command is sent directly to the joints.

#### Pupper-Walk-Relative-[Headless|Graphical]-v0

In this env, your actions are relative to the resting position (see image at the top - roughly that). Meaning an action of `[0]*12` will put the Pupper to a stable rest. Action clipping is done after summing the current action and the action corresponding to the resting position, which means the action space is asymmetric - e.g. if a given joint's resting position is at `0.7`, then the action space for that joint is `[-1.7,.3]`. This is intentional because it allows the Pupper to start with a stable position.

