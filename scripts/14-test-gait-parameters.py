import time
import numpy as np
import matplotlib.pyplot as plt
from stanford_quad.common.Utilities import controller_to_sim
from stanford_quad.pupper.policy import HardPolicy
from stanford_quad.sim.simulator2 import PupperSim2


FPS = 60  # fraction of 240
substeps = int(round(240 / FPS))

sim = PupperSim2(debug=True, frequency=240)


def reset_sim():
    sim.reset(rest=True)
    sim.change_color((0, 1, 1, 1))
    for _ in range(10):
        sim.step()


def plot_joints(joints):
    joints = np.array(joints)
    for i in range(12):
        plt.plot(np.arange(len(joints)), joints[:, i], label=f"joint {i + 1}")
    plt.legend()
    plt.tight_layout()
    plt.title("sim direct joints")
    plt.show()


reset_sim()

debug_params = []

for param in ["velocity fwd", "velocity left"]:
    debug_params.append(sim.p.addUserDebugParameter(param, -1, 1, 0))
debug_params.append(sim.p.addUserDebugParameter("reset", 1, 0, 1))
debug_params.append(sim.p.addUserDebugParameter("plot joints", 1, 0, 1))

policy = HardPolicy()
last_reset_val = -1
last_plot_val = -1
joints = []
counter = 0

while True:
    vel = [0, 0]
    for param_idx, param in enumerate(debug_params):
        val = sim.p.readUserDebugParameter(param)
        if param_idx < 2:
            vel[param_idx] = val
        if param_idx == 2 and val != last_reset_val:
            reset_sim()
            last_reset_val = val
        if param_idx == 3 and val != last_plot_val:
            if len(joints) > 0:
                plot_joints(joints)
            last_plot_val = val
            joints.clear()

    action = controller_to_sim(policy.act(velocity_horizontal=vel))
    joints.append(np.copy(action))
    sim.action(action)

    for _ in range(substeps):
        sim.step()

    time.sleep(1 / FPS)
