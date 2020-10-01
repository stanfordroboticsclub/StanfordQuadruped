import pybullet
import pybullet_data
import sim  # from sim import sim
from stanford_quadruped_controller import hardware_interface
import kinematics  # from sim import kinematics


class SimInterface(hardware_interface.HardwareInterface):
    def __init__(self, interface_config, kp=0.25, kv=0.5, max_torque=1):
        self.kp = kp
        self.kv = kv
        self.max_torque = max_torque

        self.sim = sim.Sim(xml_path="pupper_pybullet_out.xml")
        self.interface_config = interface_config

    def command_foot_positions(self, foot_positions):
        serial_joint_angles = kinematics.serial_quadruped_inverse_kinematics(
            foot_positions, self.interface_config
        )
        pybullet.setJointMotorControlArray(
            bodyUniqueId=self.sim.body_id,
            jointIndices=self.sim.joint_indices,
            controlMode=pybullet.POSITION_CONTROL,
            targetPositions=list(serial_joint_angles.T.reshape(12)),
            positionGains=[self.kp] * 12,
            velocityGains=[self.kv] * 12,
            forces=[self.max_torque] * 12,
        )

    def activate(self):
        pybullet.resetBasePositionAndOrientation(self.sim.body_id, [0,0,0.3],[0,0,0,1.0])

    def deactivate(self):
        pybullet.resetBasePositionAndOrientation(self.sim.body_id, [0,0,0.3],[0,0,0,1.0])
