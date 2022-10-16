from pupper_controller.src.pupperv3 import pupper
import math
import time
import rclpy
"""
TODO: Control-C causes an error. Says ROS wasn't shut down
"""


def run_example():
    pup = pupper.Pupper()
    pup.reset()
    print("starting...")
    pup.slow_stand(duration=0.2, do_sleep=True)
    pup.start_trot()
    last_control = pup.time()
    try:
        while True:
            # Busy-wait until it's time to run the control loop again
            while pup.time() - last_control < pup.config.dt:
                # Sleep for less time if your sim is running > 10x realtime
                time.sleep(0.0005)
            last_control = pup.time()

            # Does not obey sim time
            # clock = rclpy.clock.ROSClock()
            # clock.sleep_for(rclpy.duration.Duration(seconds=2.0)) # uses wallclock even if rosclock
            # print(clock.now().nanoseconds/1e9)

            # Run the control loop
            observation = pup.get_observation()
            pup.step(
                action={
                    "x_velocity": 0.5,
                    "y_velocity": 0.0,
                    "height": -0.18,
                    "com_x_shift": 0.005
                })

    finally:
        pup.shutdown()


if __name__ == "__main__":
    run_example()
