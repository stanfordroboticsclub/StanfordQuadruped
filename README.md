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

There are currently 11 environments. All come in 2 variants: `Headless` and `Graphical`. `Headless` is meant for training and launches the PyBullet sim without any GUI, `Graphical` is meant for local debugging, offers a GUI to inspect the robot and is significantly slower.

You can try out one of the walking environments, by running:

    python scripts/08-run-walker-gym-env.py

### Walking

In all environments, the observations are the same:

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

#### Parameters

There are several settings for the walking environment and a handful of combinations of these parameters have been given dedicated names. Here's the list of parameters and their meaning:

- `debug` (bool): If True, this will turn ON the GUI. Usually the `[Headless|Graphical]` name in the environment specifies that this is False/True respectively.
- `steps` (int): default 120, Length of an episode. This corresponds to 2 seconds at 60Hz.
- `relative_action` (bool): If False, action commands correspond directly to the joint positions. If True, then at each step, the actions are added to the stable resting position, i.e. instead of `robot.move(action)`, it's `robot.move(REST_POSE+action)`.
- `action_scaling` (float): By default, the robot has a large movement range and very responsive joints, meaning the policy can pick the maximum negative joint position in one step and the maximum positive joint position in the next step. This causes a lot of jitter. In order to reduce this, this setting allows to restrict the movement range. Best used in combination with `relative_action`.
- `action_smoothing` (int): Another method to reduce jitter. If this is larger than 1, actions aren't applied to the robot directly anymore but instead go into a queue of this length. At each step, the mean of this queue is applied to the robot.
- `random_rot` (3-tuple of floats): This allows to specify the initial random rotation. The 3 elements in the triple correspond to rotation around the x/y/z axes respectively. The rotations are drawn from a normal distribution centered at 0 and this value specifies the variance on each axis. Values are in degrees.
- `reward_stability` (float): Specifies the coefficient of the IMU reward term that encourages stability. By default, it's 0. 

Based on our previous experiments, the following set of parameters seem to perform best (corresponding to the environment **`Pupper-Walk-Relative-ScaledNSmoothed3-RandomZRot-Headless-v0`**):

```python
params = {
    "debug": False,
    "steps": 120,
    "relative_action": True,
    "action_scaling": 0.3,
    "action_smoothing": 3,
    "random_rot": (0,0,15),
    "reward_stability": 0
}
```       

#### Pupper-Walk-Absolute-[Headless|Graphical]-v0

In this environment, you have full absolute control over all joints and their resting position is set to 0 (which looks unnatural - the robot is standing fully straight, legs extended). Any action command is sent directly to the joints.

#### Pupper-Walk-Relative-[Headless|Graphical]-v0

In this env, your actions are relative to the resting position (see image at the top - roughly that). Meaning an action of `[0]*12` will put the Pupper to a stable rest. Action clipping is done after summing the current action and the action corresponding to the resting position, which means the action space is asymmetric - e.g. if a given joint's resting position is at `0.7`, then the action space for that joint is `[-1.7,.3]`. This is intentional because it allows the Pupper to start with a stable position.

#### Pupper-Walk-Relative-ScaledDown_[0.05|0.1|0.15|...|0.5]-[Headless|Graphical]-v0

Similar to the `Pupper-Walk-Relative` but here the actions are multiplied with a factor (in the environment name) to reduce the range of motion.

#### Pupper-Walk-Relative-ScaledDown-RandomZRot-[Headless|Graphical]-v0

Like the `Pupper-Walk-Relative-ScaledDown_0.3` but with random initial z rotation (rotation is drawn from a normal distribution, centered at 0, variance of 15 degrees).

#### Pupper-Walk-Relative-ScaledNSmoothed3-RandomZRot-[Headless|Graphical]-v0

Like `Pupper-Walk-Relative-ScaledDown-RandomZRot` but with an additional action smoothing of **3**.

#### Pupper-Walk-Relative-ScaledNSmoothed5-RandomZRot-[Headless|Graphical]-v0

Like `Pupper-Walk-Relative-ScaledDown-RandomZRot` but with an additional action smoothing of **5**.

#### Pupper-Walk-Relative-Smoothed5-RandomZRot-[Headless|Graphical]-v0

Like `Pupper-Walk-Relative-ScaledNSmoothed5-RandomZRot` but with the actions only averaged in a list of 5, not scaled.

#### Pupper-Walk-Relative-RewardStable0.5-[Headless|Graphical]-v0

Like `Pupper-Walk-Relative` but with the additional reward term for body orientation close to zero. Coefficient for the reward term is 0.5

#### Pupper-Walk-Relative-RewardStable0.5-ScaledDown3-[Headless|Graphical]-v0

Like `Pupper-Walk-Relative-RewardStable0.5` but additionally with the actions scaled by 0.3

#### Pupper-Walk-Relative-RewardStable0.5-ScaledDown-RandomZRot-[Headless|Graphical]-v0

Like `Pupper-Walk-Relative-RewardStable0.5-ScaledDown3` but with additional random initial rotation around the z axis

#### Pupper-Walk-Relative-RewardStable0.5-ScaledNSmoothed-RandomZRot-[Headless|Graphical]-v0

Like `Pupper-Walk-Relative-RewardStable0.5-ScaledDown-RandomZRot` but additionally with the actions smoothed with queue length 3.



