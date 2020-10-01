import pybullet


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
        self.client_id = pybullet.connect(
            pybullet.GUI
        )  # or p.DIRECT for non-graphical version
        pybullet.setGravity(0, 0, g)

        self.all_body_ids = pybullet.loadMJCF(xml_path)
        self.body_id = self.all_body_ids[1]
        print("Pupper body IDs:", self.all_body_ids)
        numjoints = pybullet.getNumJoints(self.body_id)
        print("Number of joints in converted MJCF: ", numjoints)
        print("Joint Info: ")
        for i in range(numjoints):
            print(pybullet.getJointInfo(self.body_id, i))
        self.joint_indices = list(range(0, 24, 2))

        # pybullet.setRealTimeSimulation(1)
        pybullet.setTimeStep(0.01)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_TINY_RENDERER, 1)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 0)

    def step(self):
        pybullet.stepSimulation()
