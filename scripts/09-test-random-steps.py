from stanford_quad.sim.simulator2 import PupperSim2

STAIRS_STEP_WIDTH = 1  # if you're standing in front of a staircase and looking up, what's the left-right distance?
STAIRS_STEP_DEPTH = 0.2  # if you're stepping onto one step, how much of your foot can fit on the step?
STAIRS_STEP_HEIGHT = 0.2  # how far for you have to go up between each step?

sim = PupperSim2(debug=True)
sim.reset()
sim.add_stairs(step_width=STAIRS_STEP_WIDTH, step_depth=STAIRS_STEP_DEPTH, step_height=STAIRS_STEP_HEIGHT)

for step in range(100000):
    # sim.action(motorPo)
    sim.step()
    print(sim.get_pos_orn_vel()[0][0])
