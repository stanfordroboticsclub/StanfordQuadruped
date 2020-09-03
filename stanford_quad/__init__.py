from gym import register
import numpy as np
import pdb

for headlessness in ["Headless", "Graphical"]:
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
            "reward_stability": 0.5,
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
            "reward_stability": 0.5,
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
            "reward_stability": 0.5,
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
            "reward_stability": 0.5,
        },
        max_episode_steps=120,
    )
