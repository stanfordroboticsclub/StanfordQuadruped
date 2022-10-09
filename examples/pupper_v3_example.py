from pupper_controller.src.pupperv3 import pupper
import math
import time
"""
TODO: Control-C causes an error. Says ROS wasn't shut down
"""


def run_example():
    pup = pupper.Pupper()
    pup.reset()
    print("starting...")
    # pup.slow_stand(do_sleep=True)
    pup.start_trot()
    try:
        while True:
            # Run the control loop
            pup.step(
                action={
                    "x_velocity": 0.5,
                    "y_velocity": 0.0,
                    "height": -0.18,
                    "com_x_shift": 0.005
                })
            ob = pup.get_observation()

            # Sleep until it's time to run the control loop again
            pup.sleep(0.2)
    finally:
        pup.shutdown()


if __name__ == "__main__":
    run_example()
