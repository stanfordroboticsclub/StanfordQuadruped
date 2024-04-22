# from pupper_controller.src.pupperv2 import pupper
from pupper_controller.src.pupperv3 import ros_interface
import math
import time
from absl import app
import json
# from absl import flags
# flags.DEFINE_bool("run_on_robot", False,
#                   "Whether to run on robot or in simulation.")
# FLAGS = flags.FLAGS
def run_example():
    
    interface = ros_interface.Interface(
            0, 0
        )

    joint_angles = None

    filename = "hop.json"

    joint_angles = dict()
    start = interface.time()

    read_data = False
    while True:
        # cur_joint_angles = interface.robot_state.joint_angles
        # time.sleep(1)
        # if the user presses enter in the terminal, set read data to true
        if input() == "":
            read_data = True
        interface.read_incoming_data()
        if read_data:
            print(interface.robot_state.joint_angles.tolist())
            joint_angles[interface.time() - start] = interface.robot_state.joint_angles.tolist()
            with open(filename, "w") as f:
                    json.dump(joint_angles, f)
        

    # pup = pupper.Pupper(run_on_robot=FLAGS.run_on_robot,
    #                     plane_tilt=0)  # -math.pi/8)
    # pup.reset()
    # if FLAGS.run_on_robot:
    #     # TODO: Put this code somewhere else to make pupper_example.py clean
    #     pup.hardware_interface.send_dict({"home": True})
    #     time.sleep(10)
    # print("starting...")
    # pup.slow_stand(do_sleep=True)

    # pup.start_trot()
    # try:
    #     while True:
    #         pup.step(action={"x_velocity": 0.4,
    #                             "y_velocity": 0.0,
    #                             "height": -0.14,
    #                             "com_x_shift": 0.005})
    #         ob = pup.get_observation()
    #         print(f"Roll: {ob[0]:0.2f} Pitch: {ob[1]:0.2f}")
    #         time.sleep(0.007)
    # finally:
    #     pup.hardware_interface.deactivate()
    #     pup.shutdown()
def main(_):
    run_example()

app.run(main)   