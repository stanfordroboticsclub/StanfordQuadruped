import os

import pybullet
import pybullet_data
import pybullet_utils.bullet_client as bc
import numpy as np

from stanford_quad.assets import ASSET_DIR
from stanford_quad.sim.HardwareInterface import HardwareInterface
from stanford_quad.sim.utils import random_bright_color, pybulletimage2numpy

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
        self,
        xml_path=None,
        debug=False,
        gain_pos=0.125,
        gain_vel=0.25,
        max_torque=1,
        g=-9.81,
        start_standing=True,
        img_size=(84, 84),
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
        self.img_size = img_size

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

        self.cam_proj = self.p.computeProjectionMatrixFOV(
            fov=90, aspect=self.img_size[0] / self.img_size[1], nearVal=0.001, farVal=10
        )

    def step(self):
        self.p.stepSimulation()

    def reset(self, rest=True):
        height = 0.3
        if rest:
            height = 0.182
        self.p.resetBasePositionAndOrientation(self.model, [0, 0, height], self.p.getQuaternionFromEuler([0, 0, 0]))
        if rest:
            action = self.get_rest_pos()
        else:
            action = [0] * 12
        self.action(action)
        self.set(action)
        # self.step() # one step to move joints into place

        for _ in range(10):
            self.step()

    @staticmethod
    def get_rest_pos():
        return REST_POS.T.flatten()

    def get_pos_orn_vel(self):
        posorn = self.p.getBasePositionAndOrientation(self.model)
        pos = np.array(posorn[0])
        orn = self.p.getEulerFromQuaternion(posorn[1])
        vel = self.p.getBaseVelocity(self.model)[0]

        return pos, orn, vel

    def get_joint_states(self):
        joint_states = self.p.getJointStates(self.model, self.joint_indices)
        joint_states = [joint[0] for joint in joint_states]
        return joint_states

    @staticmethod
    def joint_sanity_check(joint_angles):
        assert len(joint_angles) == 12
        assert np.min(np.rad2deg(joint_angles)) >= -180
        assert np.max(np.rad2deg(joint_angles)) <= 180

    def action(self, joint_angles):
        self.joint_sanity_check(joint_angles)

        self.p.setJointMotorControlArray(
            bodyUniqueId=self.model,
            jointIndices=self.joint_indices,
            controlMode=self.p.POSITION_CONTROL,
            targetPositions=list(joint_angles),
            positionGains=[self.gain_pos] * 12,
            velocityGains=[self.gain_vel] * 12,
            forces=[self.max_torque] * 12,
        )

    def set(self, joint_angles):
        self.joint_sanity_check(joint_angles)

        for joint_idx in range(12):
            self.p.resetJointState(
                bodyUniqueId=self.model,
                jointIndex=self.joint_indices[joint_idx],
                targetValue=joint_angles[joint_idx],
                targetVelocity=0,
            )

    def create_box(self, pos, orn, size, color, random_color):
        # we need to round or small float errors will explode the simulation
        pos = np.around(pos, 4)
        size = np.around(size, 4)
        orn = np.around(orn, 4)

        if random_color:
            color = random_bright_color(uint=False)

        obj_visual = self.p.createVisualShape(shapeType=self.p.GEOM_BOX, rgbaColor=list(color) + [1], halfExtents=size)
        obj_collision = self.p.createCollisionShape(shapeType=self.p.GEOM_BOX, halfExtents=size)

        obj = self.p.createMultiBody(
            baseMass=0.1,  # doesn't matter
            baseCollisionShapeIndex=obj_collision,
            baseVisualShapeIndex=obj_visual,
            basePosition=pos,
            baseOrientation=orn,
            useMaximalCoordinates=False,
        )
        return obj

    def add_stairs(
        self,
        no_steps=8,
        step_width=1,
        step_height=0.1,
        step_depth=0.1,
        offset=None,
        color=(0.5, 0, 0.5),
        random_color=False,
    ) -> list:
        pos_x = 0
        pos_y = 0
        pos_z = 0
        if offset is not None:
            assert len(offset) == 2 or len(offset) == 3
            pos_x += offset[0]
            pos_y += offset[1]
            if len(offset) == 3:
                pos_z += offset[2]

        steps = []
        positions = []

        orientation = self.p.getQuaternionFromEuler([0, 0, 0])
        size = (step_depth / 2, step_width / 2, step_height / 2)

        # create steps boxes
        for step_idx in range(no_steps):
            pos = (pos_x + step_depth / 2, pos_y, pos_z + size[2])

            positions.append(np.array(pos))

            step = self.create_box(pos, orientation, size, color, random_color)
            steps.append(step)

            pos_x += step_depth
            size = (size[0], size[1], size[2] + step_height / 2)

        # connect the steps to each other
        for step_idx in range(no_steps - 1):
            step_1 = steps[step_idx]
            step_2 = steps[step_idx + 1]

            pos_1 = positions[step_idx]
            pos_2 = positions[step_idx + 1]

            # in order to create a fixing constraint between parent and child step,
            # you need to find a point where they touch.
            center = (pos_1 + pos_2) / 2
            parent_frame = center - pos_1
            child_frame = center - pos_2

            self.p.createConstraint(
                parentBodyUniqueId=step_1,
                parentLinkIndex=-1,
                childBodyUniqueId=step_2,
                childLinkIndex=-1,
                jointType=self.p.JOINT_FIXED,
                jointAxis=(1, 1, 1),
                parentFramePosition=parent_frame,
                childFramePosition=child_frame,
            )

        # and fix the first step to the ground/world
        self.p.createConstraint(
            parentBodyUniqueId=steps[0],
            parentLinkIndex=-1,
            childBodyUniqueId=-1,
            childLinkIndex=-1,
            jointType=self.p.JOINT_FIXED,
            jointAxis=(1, 1, 1),
            parentFramePosition=-positions[0],
            childFramePosition=(0, 0, 0),
        )

        return steps

    def take_photo(self):
        pos, _, _ = self.get_pos_orn_vel()
        cam_pos = pos + [0, -0.3, 0.3]
        cam_view = self.p.computeViewMatrix(
            cameraEyePosition=cam_pos, cameraTargetPosition=pos, cameraUpVector=[0, 0, 1]
        )
        img = self.p.getCameraImage(
            self.img_size[0],
            self.img_size[1],
            cam_view,
            self.cam_proj,
            renderer=self.p.ER_BULLET_HARDWARE_OPENGL,
            flags=0
            # lightDirection=[-.5, -1, .5], lightDistance=1,
            # renderer=self.p0.ER_TINY_RENDERER
        )
        output_img = pybulletimage2numpy(img, self.img_size[0], self.img_size[1])
        return output_img


# depth = 0.2, height = 0.2
# pos 0.1, 0.1, size 0.1, 0.1, total height 0.2
# pos 0.2, 0.2, size 0.1, 0.2, total height 0.4
#
