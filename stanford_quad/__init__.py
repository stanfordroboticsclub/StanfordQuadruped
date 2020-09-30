from gym import register
import numpy as np

from stanford_quad.envs.imitation_recordings import IMITATION_LIB

HEADLESSNESS = ["Headless", "Graphical"]


def make_imitation_env(trick):
    steps = IMITATION_LIB[trick]["end"] - IMITATION_LIB[trick]["start"]
    register(
        id=f"Pupper-Recording-{IMITATION_LIB[trick]['env_name']}-v0",
        entry_point="stanford_quad.envs:ImitationEnv",
        kwargs={
            # "debug": (False if headlessness == "Headless" else True),
            "trick": "walk-forward",
        },
        max_episode_steps=steps,
    )


make_imitation_env("walk-forward")

for headlessness in HEADLESSNESS:
    register(
        id=f"Pupper-Walk-Absolute-{headlessness}-v0",
        entry_point="stanford_quad.envs:WalkingEnv",
        kwargs={
            "debug": (False if headlessness == "Headless" else True),
            "steps": 120,
            "relative_action": False,
            "action_scaling": 1,
        },
        max_episode_steps=120,
    )
    register(
        id=f"Pupper-Walk-Relative-{headlessness}-v0",
        entry_point="stanford_quad.envs:WalkingEnv",
        kwargs={
            "debug": (False if headlessness == "Headless" else True),
            "steps": 120,
            "relative_action": True,
            "action_scaling": 1,
        },
        max_episode_steps=120,
    )
    for scale_down in list(np.arange(0.05, 0.5, 0.05)):
        register(
            id=f"Pupper-Walk-Relative-ScaledDown_{scale_down:.2}-{headlessness}-v0",
            entry_point="stanford_quad.envs:WalkingEnv",
            kwargs={
                "debug": (False if headlessness == "Headless" else True),
                "steps": 120,
                "relative_action": True,
                "action_scaling": scale_down,
            },
            max_episode_steps=120,
        )
        # print(f"Pupper-Walk-Relative-ScaledDown_{scale_down:.2}-{headlessness}-v0")
    register(
        id=f"Pupper-Walk-Relative-ScaledDown-RandomZRot-{headlessness}-v0",
        entry_point="stanford_quad.envs:WalkingEnv",
        kwargs={
            "debug": (False if headlessness == "Headless" else True),
            "steps": 120,
            "relative_action": True,
            "action_scaling": 0.3,
            "random_rot": (0, 0, 15),
        },
        max_episode_steps=120,
    )
    register(
        id=f"Pupper-Walk-Relative-ScaledNSmoothed3-RandomZRot-{headlessness}-v0",
        entry_point="stanford_quad.envs:WalkingEnv",
        kwargs={
            "debug": (False if headlessness == "Headless" else True),
            "steps": 120,
            "relative_action": True,
            "action_scaling": 0.3,
            "action_smoothing": 3,
            "random_rot": (0, 0, 15),
        },
        max_episode_steps=120,
    )
    register(
        id=f"Pupper-Walk-Relative-ScaledNSmoothed5-RandomZRot-{headlessness}-v0",
        entry_point="stanford_quad.envs:WalkingEnv",
        kwargs={
            "debug": (False if headlessness == "Headless" else True),
            "steps": 120,
            "relative_action": True,
            "action_scaling": 0.3,
            "action_smoothing": 5,
            "random_rot": (0, 0, 15),
        },
        max_episode_steps=120,
    )
    register(
        id=f"Pupper-Walk-Relative-Smoothed5-RandomZRot-{headlessness}-v0",
        entry_point="stanford_quad.envs:WalkingEnv",
        kwargs={
            "debug": (False if headlessness == "Headless" else True),
            "steps": 120,
            "relative_action": True,
            "action_scaling": 1,
            "action_smoothing": 5,
            "random_rot": (0, 0, 15),
        },
        max_episode_steps=120,
    )
    register(
        id=f"Pupper-Walk-Relative-RewardStable0.5-{headlessness}-v0",
        entry_point="stanford_quad.envs:WalkingEnv",
        kwargs={
            "debug": (False if headlessness == "Headless" else True),
            "steps": 120,
            "relative_action": True,
            "action_scaling": 1,
            "reward_coefficients": (0.1, 1, 0.5),
        },
        max_episode_steps=120,
    )
    register(
        id=f"Pupper-Walk-Relative-RewardStable0.5-ScaledDown3-{headlessness}-v0",
        entry_point="stanford_quad.envs:WalkingEnv",
        kwargs={
            "debug": (False if headlessness == "Headless" else True),
            "steps": 120,
            "relative_action": True,
            "action_scaling": 0.3,
            "reward_coefficients": (0.1, 1, 0.5),
        },
        max_episode_steps=120,
    )
    register(
        id=f"Pupper-Walk-Relative-RewardStable0.5-ScaledDown-RandomZRot-{headlessness}-v0",
        entry_point="stanford_quad.envs:WalkingEnv",
        kwargs={
            "debug": (False if headlessness == "Headless" else True),
            "steps": 120,
            "relative_action": True,
            "action_scaling": 0.3,
            "random_rot": (0, 0, 15),
            "reward_coefficients": (0.1, 1, 0.5),
        },
        max_episode_steps=120,
    )
    register(
        id=f"Pupper-Walk-Relative-RewardStable0.5-ScaledNSmoothed-RandomZRot-{headlessness}-v0",
        entry_point="stanford_quad.envs:WalkingEnv",
        kwargs={
            "debug": (False if headlessness == "Headless" else True),
            "steps": 120,
            "relative_action": True,
            "action_scaling": 0.3,
            "action_smoothing": 3,
            "random_rot": (0, 0, 15),
            "reward_coefficients": (0.1, 1, 0.5),
        },
        max_episode_steps=120,
    )

RELATIVE_ACTION = [True, False]
ACTION_SMOOTHING = [1, 2, 3, 4]
RANDOM_ROT = [0, 1, 10, 100]
ACTION_SCALING = [1.0] + list(np.arange(0.05, 0.5, 0.05))

for headlessness in HEADLESSNESS:
    for relative_action in RELATIVE_ACTION:
        for action_smoothing in ACTION_SMOOTHING:
            for action_scaling in ACTION_SCALING:
                for random_rot in RANDOM_ROT:
                    action = "Relative" if relative_action else "Absolute"
                    register(
                        id=f"Pupper-Walk-{action}-"
                        f"aScale_{action_scaling:.2}-"
                        f"aSmooth_{action_smoothing}-"
                        f"RandomZRot_{random_rot}-{headlessness}-v0",
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
