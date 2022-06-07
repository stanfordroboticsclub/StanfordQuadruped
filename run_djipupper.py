# Usage
# python run_djipupper.py --run_on_robot

from pupper_controller.src.pupperv2 import pupper
from pupper_controller.src.interfaces import JoystickInterface
import math
import time
from absl import app
from absl import flags
flags.DEFINE_bool("run_on_robot", False,
                  "Whether to run on robot or in simulation.")
FLAGS = flags.FLAGS
def run_controller():
    pup = pupper.Pupper(run_on_robot=FLAGS.run_on_robot,
                        plane_tilt=0)  # -math.pi/8)

    timestep = 0.01 # s
    last_time = 0
    iteration = 0
    mode = "stand"

    pup.reset()
    if (FLAGS.run_on_robot):
        pup.hardware_interface.send_dict({"home": True})
        time.sleep(6)
    print("starting...")
    pup.slow_stand(do_sleep=True)

    joystick_interface = JoystickInterface.JoystickInterface(pup.config)
    try:
        while True:
            cur_time = time.time()  # s
            if (cur_time > last_time + timestep):
                last_time = cur_time
                command = joystick_interface.get_command(pup.state)
                if (command.trot_event):
                    mode = "trot"
                    pup.start_trot()
                if (command.stand_event):
                    mode = "stand"
                    pup.stop_trot()
                if (command.walk_event):
                    mode = "walk"

                print(mode)

                if (mode == "trot"):
                    pup.step(action={"x_velocity": command.horizontal_velocity[0],
                                     "y_velocity": command.horizontal_velocity[1],
                                     "yaw_rate": command.yaw_rate,
                                     "height": command.height,
                                     "com_x_shift": 0.005})
                if (mode == "stand"):
                    pup.step({'height': command.height})
                if (mode == "walk"):
                    print("Mode: walk not implemented")


                if(iteration % 100 == 0):
                    print("Time_step:", time.time() - last_time)
                iteration += 1
                state = pup.get_observation()
    finally:
        pup.shutdown()
def main(_):
    run_controller()
if __name__=='__main__':
    app.run(main)