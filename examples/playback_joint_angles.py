# from pupper_controller.src.pupperv2 import pupper
from pupper_controller.src.pupperv3 import ros_interface
import math
import time
from absl import app
import json
import numpy as np

def run_example():

    joint_angles = dict()

    interface = ros_interface.Interface(
            0, 0
        )

    with open(f"standup3.json", "r") as f:
        joint_angles = json.load(f)

    prev = 0.
    
    cur_angles = None
    while True:
        interface.read_incoming_data()
        cur_angles = interface.robot_state.joint_angles
        # breakpoint()
        print(cur_angles)
        if isinstance(cur_angles, np.ndarray):
            if not np.any(cur_angles == None):
                break
        time.sleep(0.1)

    print("angs after waiting", cur_angles)

    for timestamp, angles in joint_angles.items(): 
        interpolation_steps = 50
        sleep_time = (float(timestamp)-prev) / interpolation_steps

        angles = np.array(angles)
        step_angles = (angles - cur_angles) / interpolation_steps
        
        for i in range(interpolation_steps):
            print(f"Setting joint angles to {angles}")

            # flip the 0 and 1 columns of the angles
            
            # angles_flip = [[angles[:,3]], [angles[:,2]], [angles[:,1]], [angles[:,0]]]
            cur_angles = cur_angles + step_angles
            interface.set_joint_angles(cur_angles)
            # convert timestamp (a string) to a float
            # time.sleep(float(timestamp)-prev)
            time.sleep(sleep_time)
            
        prev = float(timestamp)


run_example()

