from pupper_controller.src.pupperv3 import pupper, ros_joystick_interface
import time
"""
TODO: Control-C causes an error. Says ROS wasn't shut down
"""


def run_example():
    joystick = ros_joystick_interface.Joystick()
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
                # Reduce sleep time if your sim runs > 10x realtime
                time.sleep(0.0005)
            last_control = pup.time()

            # Run the control loop
            observation = pup.get_observation()
            joystick_vals = joystick.joystick_values()
            print("received joystick values: ", joystick_vals)
            pup.step(
                action={
                    "x_velocity": joystick_vals["left_y"] / 1.5,
                    "y_velocity": -joystick_vals["left_x"] / 1.5,
                    "yaw_rate": -joystick_vals["right_x"] * 4,
                    "height": -0.18,
                    "com_x_shift": 0.005
                })

    finally:
        pup.shutdown()


if __name__ == "__main__":
    run_example()
