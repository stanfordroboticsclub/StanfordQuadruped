import os

import pybullet
import pybullet_data
import pybullet_utils.bullet_client as bc
import numpy as np

from stanford_quad.assets import ASSET_DIR
from stanford_quad.sim.HardwareInterface import HardwareInterface

FREQ_SIM = 240

REST_POS = HardwareInterface.parallel_to_serial_joint_angles(
    np.array(
        [
            [-0.12295051, 0.12295051, -0.12295051, 0.12295051],
            [0.77062617, 0.77062617, 0.77062617, 0.77062617],
            [-0.845151, -0.845151, -0.845151, -0.845151],
        ]
    )
)


class PupperSim2:
    def __init__(
        self, xml_path=None, debug=False, gain_pos=0.125, gain_vel=0.25, max_torque=1, g=-9.81, start_standing=True
    ):
        """

        :param str xml_path: Path to Mujoco robot XML definition file
        :param bool debug: Set to True to enable visual inspection
        :param float gain_pos: Gain position value for low-level controller
        :param float gain_vel: Gain velocity value for low-level controller
        :param float max_torque: Max torque for PID controller
        :param float g: Gravity value
        """
        if xml_path is None:
            xml_path = os.path.join(ASSET_DIR, "pupper_pybullet_out.xml")

        self.gain_pos = gain_pos
        self.gain_vel = gain_vel
        self.max_torque = max_torque

        if debug:
            startup_flag = pybullet.GUI
        else:
            startup_flag = pybullet.DIRECT
        self.p = bc.BulletClient(connection_mode=startup_flag)
        self.p.setTimeStep(1 / FREQ_SIM)
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.p.setGravity(0, 0, g)

        # self.p.loadURDF("plane.urdf")

        self.model = self.p.loadMJCF(xml_path)[1]  # [0] is the floor
        if debug:
            numjoints = self.p.getNumJoints(self.model)
            for i in range(numjoints):
                print(self.p.getJointInfo(self.model, i))
        self.joint_indices = list(range(0, 24, 2))

        if start_standing:
            self.reset()
            self.step()

    def step(self):
        self.p.stepSimulation()

    def reset(self, rest=True):
        self.p.resetBasePositionAndOrientation(self.model, [0, 0, .3], self.p.getQuaternionFromEuler([0, 0, 0]))
        if rest:
            action = self.get_rest_pos()
        else:
            action = [0] * 12
        self.action(action)
        self.step() # one step to move joints into place

    def get_rest_pos(self):
        return REST_POS.T.flatten()

    def get_pos_orn_vel(self):
        posorn = self.p.getBasePositionAndOrientation(self.model)
        pos = posorn[0]
        orn = self.p.getEulerFromQuaternion(posorn[1])
        vel = self.p.getBaseVelocity(self.model)[0]

        return pos, orn, vel

    def get_joint_states(self):
        joint_states = self.p.getJointStates(self.model, self.joint_indices)
        joint_states = [joint[0] for joint in joint_states]
        return joint_states

    def action(self, joint_angles):
        assert len(joint_angles) == 12
        assert np.min(np.rad2deg(joint_angles)) >= -180
        assert np.max(np.rad2deg(joint_angles)) <= 180

        self.p.setJointMotorControlArray(
            bodyUniqueId=self.model,
            jointIndices=self.joint_indices,
            controlMode=self.p.POSITION_CONTROL,
            targetPositions=list(joint_angles),
            positionGains=[self.gain_pos] * 12,
            velocityGains=[self.gain_vel] * 12,
            forces=[self.max_torque] * 12,
        )
