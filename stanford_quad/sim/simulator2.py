import os
import time

import pybullet
import pybullet_data
import pybullet_utils.bullet_client as bc
import numpy as np

from stanford_quad.assets import ASSET_DIR
from stanford_quad.sim.HardwareInterface import HardwareInterface
from stanford_quad.sim.utils import random_bright_color, pybulletimage2numpy, pybulletsegmap2numpy

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
        enable_new_floor=False,
        frequency=FREQ_SIM,
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
        self.p.setTimeStep(1 / frequency)
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.p.setGravity(0, 0, g)

        if debug:
            self.p.resetDebugVisualizerCamera(
                cameraDistance=1.5, cameraYaw=180, cameraPitch=-45, cameraTargetPosition=[0, 0.0, 0]
            )

        # self.p.loadURDF("plane.urdf")

        self.floor, self.model = self.p.loadMJCF(xml_path)

        if debug:
            numjoints = self.p.getNumJoints(self.model)
            for i in range(numjoints):
                print(self.p.getJointInfo(self.model, i))
        self.joint_indices = list(range(0, 24, 2))

        self.cam_proj = self.p.computeProjectionMatrixFOV(
            fov=90, aspect=self.img_size[0] / self.img_size[1], nearVal=0.001, farVal=10
        )

        self.collision_floor = None
        if enable_new_floor:
            self.collision_floor = self.create_box((0, 0, 0.001), (0, 0, 0), (1, 1, 0.0005), visible=False)
            self.p.createConstraint(
                parentBodyUniqueId=self.collision_floor,
                parentLinkIndex=-1,
                childBodyUniqueId=-1,
                childLinkIndex=-1,
                jointType=self.p.JOINT_FIXED,
                jointAxis=(1, 1, 1),
                parentFramePosition=(0, 0, 0.0),
                childFramePosition=(0, 0, 0.0005),
            )

        if start_standing:
            self.reset(True)
            for _ in range(10):
                self.step()

        # time.sleep(1)

    def step(self):
        self.p.stepSimulation()

    def reset(self, rest=True, random_rot=(0, 0, 0), random_pos=(0, 0, 0)):
        height = 0.301
        if rest:
            height = 0.183
        # self.p.resetBasePositionAndOrientation(self.model, [0, 0, height], self.p.getQuaternionFromEuler([0, 0, 0]))
        rand_pos = (
            np.random.uniform(-random_pos[0], random_pos[0], 1)[0],
            np.random.uniform(-random_pos[1], random_pos[1], 1)[0],
            np.random.uniform(-random_pos[2], random_pos[2], 1)[0] + height,
        )
        rand_rot = (
            np.random.normal(0, random_rot[0], 1)[0],
            np.random.normal(0, random_rot[1], 1)[0],
            np.random.normal(0, random_rot[2], 1)[0],
        )
        rand_rot = [np.deg2rad(x) for x in rand_rot]

        self.p.resetBasePositionAndOrientation(
            self.model, rand_pos, self.p.getQuaternionFromEuler(rand_rot),
        )
        if rest:
            action = self.get_rest_pos()
        else:
            action = [0] * 12
        self.action(action)
        self.set(action)
        # self.step() # one step to move joints into place

        for _ in range(10):
            self.step()

    def make_kinematic(self, color=(1, 0, 1, 1)):
        sleepy_state = (
            self.p.ACTIVATION_STATE_SLEEP
            + self.p.ACTIVATION_STATE_ENABLE_SLEEPING
            + self.p.ACTIVATION_STATE_DISABLE_WAKEUP
        )

        self.p.changeDynamics(self.model, -1, linearDamping=0, angularDamping=0)
        self.p.setCollisionFilterGroupMask(self.model, -1, collisionFilterGroup=0, collisionFilterMask=0)

        self.p.changeDynamics(
            self.model, -1, activationState=sleepy_state,
        )
        for j in range(24):  # not 12 because there's some fixed joints in there
            self.p.setCollisionFilterGroupMask(self.model, j, collisionFilterGroup=0, collisionFilterMask=0)

            self.p.changeDynamics(
                self.model, j, activationState=sleepy_state,
            )

        self.change_color(color)

    def change_color(self, color=(1, 0, 1, 1)):
        self.p.changeVisualShape(self.model, -1, rgbaColor=color)
        for j in range(24):  # not 12 because there's some fixed joints in there
            self.p.changeVisualShape(self.model, j, rgbaColor=color)

    def move_kinectic_body(self, pos, rot):
        self.p.resetBasePositionAndOrientation(self.model, pos, rot)

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

    def create_box(self, pos, orn, size, color=(1, 0, 0), random_color=False, visible=True):
        # we need to round or small float errors will explode the simulation
        pos = np.around(pos, 4)
        size = np.around(size, 4)
        orn = np.around(self.p.getQuaternionFromEuler(orn), 4)

        if random_color:
            color = random_bright_color(uint=False)

        obj_visual = -1
        if visible:
            obj_visual = self.p.createVisualShape(
                shapeType=self.p.GEOM_BOX, rgbaColor=list(color) + [1], halfExtents=size
            )

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

        orientation = [0, 0, 0]
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

    def take_photo(self, camera_offset=(0, -0.3, 0.3), with_segmap=False):
        pos, _, _ = self.get_pos_orn_vel()
        cam_pos = pos + camera_offset
        cam_view = self.p.computeViewMatrix(
            cameraEyePosition=cam_pos, cameraTargetPosition=pos, cameraUpVector=[0, 0, 1]
        )
        img = self.p.getCameraImage(
            self.img_size[0],
            self.img_size[1],
            cam_view,
            self.cam_proj,
            renderer=pybullet.ER_BULLET_HARDWARE_OPENGL,
            # flags=pybullet.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX
            # lightDirection=[-.5, -1, .5], lightDistance=1,
            # renderer=self.p0.ER_TINY_RENDERER
        )
        output_img = pybulletimage2numpy(img)

        if with_segmap:
            output_segmap = pybulletsegmap2numpy(img)
            return output_img, output_segmap
        else:
            return output_img

    def check_collision(self):
        if self.collision_floor is None:
            raise Exception(
                "You need to call the PupperSim2 class with the parameter enable_new_floor=True for floor collisions to work."
            )

        contacts = self.p.getClosestPoints(
            bodyA=self.model, bodyB=self.collision_floor, distance=0.1, linkIndexA=-1, linkIndexB=-1
        )
        # if len(contacts) > 0:
        #     contacts = contacts[0]
        #     self.p.removeAllUserDebugItems()
        #     self.p.addUserDebugLine(lineFromXYZ=contacts[5], lineToXYZ=contacts[6], lineColorRGB=(1, 0, 1), lineWidth=2)
        #     self.p.addUserDebugText(text=str(contacts[8]), textPosition=(0, 0, 0), textColorRGB=(1, 1, 0), textSize=5)
        if len(contacts) > 0 and contacts[0][8] < 0.02:
            return True

        return False


# depth = 0.2, height = 0.2
# pos 0.1, 0.1, size 0.1, 0.1, total height 0.2
# pos 0.2, 0.2, size 0.1, 0.2, total height 0.4
#
