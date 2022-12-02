from pupper_controller.src.pupperv3 import pupper
import time
"""
TODO: Control-C causes an error. Says ROS wasn't shut down
"""


def run_example():
    pup = pupper.Pupper()
    pup.reset()
    print("starting...")
    pup.slow_stand(min_height=-0.06, duration=0.2, do_sleep=True)
    pup.start_trot()
    last_control = pup.time()
    try:
        while True:
            # Busy-wait until it's time to run the control loop again
            while pup.time() - last_control < pup.config.dt:
                # Reduce sleep time if your sim runs > 10x realtime
                time.sleep(0.0005)
            last_control = pup.time()

            # Run the control loop
            observation = pup.get_observation()
            pup.step(
                action={
                    "x_velocity": 0.0,
                    "y_velocity": 0.0,
                    "yaw_rate": 0.0,
                    "height": -0.18,
                    "com_x_shift": 0.005
                })

    finally:
        pup.shutdown()


if __name__ == "__main__":
    run_example()
