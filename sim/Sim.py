import pybullet
import pybullet_data


class Sim:
    def __init__(
        self,
        xml_path,
        kp=0.25,
        kv=0.5,
        max_torque=10,
        g=-9.81,
    ):
        # Set up PyBullet Simulator
        pybullet.connect(pybullet.GUI)  # or p.DIRECT for non-graphical version
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        pybullet.setGravity(0, 0, g)
        self.model = pybullet.loadMJCF(xml_path)
        print("")
        print("Pupper body IDs:", self.model)
        numjoints = pybullet.getNumJoints(self.model[1])
        print("Number of joints in converted MJCF: ", numjoints)
        print("Joint Info: ")
        for i in range(numjoints):
            print(pybullet.getJointInfo(self.model[1], i))
        self.joint_indices = list(range(0, 24, 2))

    def step(self):
        pybullet.stepSimulation()
