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
    pup.slow_stand(min_height=-0.06, duration=1.0, do_sleep=True)
    # pup.start_trot()
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
            if joystick_vals["L2"] < 0:
                behavior_state_override = "trot"
            else:
                behavior_state_override = "rest"
            print("received joystick values: ", joystick_vals)
            pup.step(action={
                "x_velocity": joystick_vals["left_y"] / 1.5,
                "y_velocity": -joystick_vals["left_x"] / 1.5,
                "yaw_rate": -joystick_vals["right_x"] * 4,
                "pitch": joystick_vals["right_y"] * -0.25,
                "height": -0.16,
                "com_x_shift": -0.01
            },
                     behavior_state_override=behavior_state_override)

    finally:
        pup.shutdown()


if __name__ == "__main__":
    run_example()
