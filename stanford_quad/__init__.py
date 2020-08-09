from gym import register

for headlessness in ["Headless", "Graphical"]:
    register(
        id=f"Pupper-Walk-Absolute-{headlessness}-v0",
        entry_point="stanford_quad.envs:WalkingEnv",
        kwargs={"debug": (False if headlessness == "Headless" else True), "steps": 120, "relative_action": False},
        max_episode_steps=120,
    )
    register(
        id=f"Pupper-Walk-Relative-{headlessness}-v0",
        entry_point="stanford_quad.envs:WalkingEnv",
        kwargs={"debug": (False if headlessness == "Headless" else True), "steps": 120, "relative_action": True},
        max_episode_steps=120,
    )
