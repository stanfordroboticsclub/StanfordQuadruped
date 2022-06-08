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

    timestep = 0.010 # s
    speed_constant = 1.33
    pup.config.dt = timestep * speed_constant
    print_time = 0.3
    last_time = 0
    iteration = 0
    it_max = math.ceil(print_time / timestep)
    skipped_steps = 0

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
                pup.step_with_command(command)

                state = pup.get_observation()

                if(iteration % it_max == 0):
                    time_spent = (time.time() - last_time)
                    print("Calculation Time:", "{0:.5f}".format(time_spent),
                          "\t Skipped Steps:", skipped_steps)
                iteration += 1
                skipped_steps = 0
            else:
                skipped_steps += 1
    finally:
        pup.shutdown()
def main(_):
    run_controller()
if __name__=='__main__':
    app.run(main)