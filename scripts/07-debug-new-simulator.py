import time
import numpy as np
from stanford_quad.sim.simulator2 import PupperSim2, REST_POS

sim = PupperSim2(debug=True)

debugParams = []

rest_pos = REST_POS.T.flatten()
for i in range(len(sim.joint_indices)):
    motor = sim.p.addUserDebugParameter("motor{}".format(i + 1), -np.pi, np.pi, rest_pos[i])
    debugParams.append(motor)

print(sim.get_pos_orn_vel())
print(sim.get_joint_states())

for step in range(100000):
    motorPos = []
    for i in range(len(sim.joint_indices)):
        pos = sim.p.readUserDebugParameter(debugParams[i])
        motorPos.append(pos)
    sim.action(motorPos)
    sim.step()
    pos, orn, vel = sim.get_pos_orn_vel()
    # print(pos, orn, vel)
    # for i in range(40):
    #     print(sim.p.getLinkState(bodyUniqueId=sim.model, linkIndex=i))
    # quit()
    sim.check_collision()
