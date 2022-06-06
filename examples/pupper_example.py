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
    if FLAGS.run_on_robot:
        # TODO: Put this code somewhere else to make pupper_example.py clean
        pup.hardware_interface.send_dict({"home": True})
        time.sleep(10)
    print("starting...")
    pup.slow_stand(do_sleep=True)

    pup.start_trot()
    try:
        while True:
            pup.step(action={"x_velocity": 0.4,
                                "y_velocity": 0.0,
                                "height": -0.14,
                                "com_x_shift": 0.005})
            ob = pup.get_observation()
            print(f"Roll: {ob[0]:0.2f} Pitch: {ob[1]:0.2f}")
            time.sleep(0.007)
    finally:
        pup.hardware_interface.deactivate()
        pup.shutdown()
def main(_):
    run_example()

app.run(main)   