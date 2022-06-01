from pupper_controller.src.pupperv2 import pupper
import math
import time
from absl import app
from absl import flags

flags.DEFINE_bool("run_on_robot", False,
                  "Whether to run on robot or in simulation.")
FLAGS = flags.FLAGS


def run_example():
    pup = pupper.Pupper(run_on_robot=FLAGS.run_on_robot,
                        plane_tilt=0)  # -math.pi/8)
    pup.reset()
    pup.slow_stand(do_sleep=True)
    pup.start_trot()

    try:
        while True:
            with open('C:\Users\user\depthai_blazepose\pupper_instructions.txt') as f:
                lines = f.readlines()
                if (lines == "left"):
                    y_velocity = -0.25
                elif (lines == "right"):
                    y_velocity = 0.25
                elif (lines == "both"):
                    x_velocity = 0.25
            pup.step(action={"x_velocity": x_velocity,
                             "y_velocity": y_velocity,
                             "height": -0.14,
                             "com_x_shift": 0.005})
            ob = pup.get_observation()
            time.sleep(0.01)
    finally:
        pup.shutdown()

def main(_):
    run_example()


if __name__ == "__main__":
    app.run(main)
