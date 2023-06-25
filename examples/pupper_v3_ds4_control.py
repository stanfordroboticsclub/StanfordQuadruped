from pupper_controller.src.pupperv3 import pupper, ros_joystick_interface
import time
import argparse

"""
TODO: Control-C causes an error. Says ROS wasn't shut down
"""


def run_example(half_robot=False):
    joystick = ros_joystick_interface.Joystick()
    pup = pupper.Pupper(half_robot=half_robot)
    pup.reset()
    print("starting...")
    pup.slow_stand(min_height=-0.11, duration=1.0, do_sleep=True)
    last_control = pup.time()
    com_x_shift = -0.02
    height = -0.15
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

            com_x_shift += joystick_vals["d_pad_y"] * pup.config.dt / 100.0
            com_x_shift = min(max(com_x_shift, -0.05), 0.05)

            height += (
                (joystick_vals["x"] - joystick_vals["triangle"]) * pup.config.dt / 25.0
            )
            height = min(max(height, -0.25), -0.05)

            if joystick_vals["connected"]:
                print(
                    f"com_x_shift: {com_x_shift:0.4f} height: {height:0.4f} joystick values: {joystick_vals}"
                )
            pup.step(
                action={
                    "x_velocity": joystick_vals["left_y"] / 1.5,
                    "y_velocity": -joystick_vals["left_x"] / 1.5,
                    "yaw_rate": -joystick_vals["right_x"] * 4,
                    "pitch": joystick_vals["right_y"] * -0.5,
                    "height": height,
                    "com_x_shift": com_x_shift,
                },
                behavior_state_override=behavior_state_override,
            )

    finally:
        pup.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--half", action="store_true", help="Enable flag half")
    args = parser.parse_args()
    run_example(half_robot=args.half)
