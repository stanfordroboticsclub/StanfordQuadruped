from gym import register
import numpy as np
import pdb


HEADLESSNESS =  ["Headless", "Graphical"]
RELATIVE_ACTION = [True, False]
ACTION_SMOOTHING = [1, 2, 3, 4]
RANDOM_ROT = [0, 1, 10, 100]
ACTION_SCALING = [1.] + list(np.arange(0.05, 0.5, 0.05)) 

for headlessness in HEADLESSNESS:
    for relative_action in RELATIVE_ACTION:
        for action_smoothing in ACTION_SMOOTHING:
            for action_scaling in ACTION_SCALING:
                for random_rot in RANDOM_ROT:
                    action = 'Relative' if relative_action else 'Absolute'
                    register(
                        id=f"Pupper-Walk-{action}_aScale_{action_scaling:.2}-aSmooth_{action_smoothing}-RandomZRot_{random_rot}-{headlessness}-v0",
                        entry_point="stanford_quad.envs:WalkingEnv",
                        kwargs={
                            "debug": (False if headlessness == "Headless" else True),
                            "steps": 120,
                            "relative_action": relative_action,
                            "action_scaling": action_scaling,
                            "action_smoothing": action_smoothing,
                            "random_rot": (0, 0, random_rot),
                        },
                        max_episode_steps=120,
                    )