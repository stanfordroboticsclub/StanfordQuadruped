from gym import register

for headlessness in ["Headless", "Graphical"]:
    register(
        id=f'Splearn-Poke-Single-{headlessness}-v0',
        entry_point='splearn.envs:PokeSingleEnv',
        kwargs={
            "headless": (True if headlessness == "Headless" else False),
            "steps": 10
        },
        max_episode_steps=10,
    )

    register(
        id=f'Splearn-Poke-RandomSand-{headlessness}-v0',
        entry_point='splearn.envs:PokeSingleEnv',
        kwargs={
            "headless": (True if headlessness == "Headless" else False),
            "steps": 10,
            "random_sand": True
        },
        max_episode_steps=10,
    )