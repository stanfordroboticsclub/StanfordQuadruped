from pupper_controller.src.pupperv3 import pupper
import time
import argparse

"""
TODO: Control-C causes an error. Says ROS wasn't shut down
"""


def run_example(half_robot: bool = False, log: bool = False, x_velocity: float = 0.0):
    pup = pupper.Pupper(half_robot=half_robot, log=log)
    pup.reset()
    print("Standing up...")
    pup.slow_stand(min_height=-0.06, duration=0.2, do_sleep=True)
    print("Starting trot...")
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
                    "x_velocity": x_velocity,
                    "y_velocity": 0.0,
                    "yaw_rate": 0.0,
                    "height": -0.18,
                    "com_x_shift": 0.005,
                }
            )

    finally:
        pup.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--half", action="store_true", help="Enable flag half")
    parser.add_argument("--log", action="store_true", help="Enable flag log")
    parser.add_argument("--x_velocity", type=float, default=0)
    args = parser.parse_args()
    if args.half:
        print("Only commanding half of robot")
    run_example(half_robot=args.half, logs=args.log, x_velocity=args.x_velocity)
