import pybullet
import pybullet_data


class HardwareInterface:
    def __init__(self, model, joint_indices, kp=0.25, kv=0.5, max_torque=10):
        self.model = model
        self.joint_indices = joint_indices
        self.kp = kp
        self.kv = kv
        self.max_torque = max_torque

    def set_actuator_postions(self, joint_angles):
        serial_joint_angles = self.parallel_to_serial_joint_angles(joint_angles)
        pybullet.setJointMotorControlArray(
            bodyUniqueId=self.model[1],
            jointIndices=self.joint_indices,
            controlMode=pybullet.POSITION_CONTROL,
            targetPositions=list(serial_joint_angles.T.reshape(12)),
            positionGains=[self.kp] * 12,
            velocityGains=[self.kv] * 12,
            forces=[self.max_torque] * 12,
        )

    def parallel_to_serial_joint_angles(self, joint_matrix):
        """Convert from joint angles meant for the parallel linkage in 
        Pupper to the joint angles in the serial linkage approximation implemented in the simulation
        
        Parameters
        ----------
        joint_matrix : Numpy array (3, 4)
            Joint angles for parallel linkage
        
        Returns
        -------
        Numpy array (3, 4)
            Joint angles for equivalent serial linkage
        """
        temp = joint_matrix
        temp[2, :] -= joint_matrix[1, :]
        return temp
