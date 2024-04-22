from pupper_controller.src.pupperv3 import pupper, ros_joystick_interface
import time
import argparse

DEFAULT_X_SHIFT = -0.0095#-0.035
DEFAULT_TROT_HEIGHT = -0.12


def run_example(half_robot=False):
    joystick = ros_joystick_interface.Joystick()
    pup = pupper.Pupper(half_robot=half_robot)
    pup.reset()
    print("starting...")
    # pup.slow_stand(min_height=-0.08, duration=1.0, do_sleep=True)
    last_control = pup.time()
    com_x_shift = DEFAULT_X_SHIFT
    height = DEFAULT_TROT_HEIGHT
    filtered_control_rate = 100.0
    alpha = 0.95
    try:
        while True:
            # Busy-wait until it's time to run the control loop again
            while pup.time() - last_control < pup.config.dt:
                # Reduce sleep time if your sim runs > 10x realtime
                time.sleep(0.0001)
            filtered_control_rate = alpha * filtered_control_rate + \
                (1-alpha) / (pup.time() - last_control)
            last_control = pup.time()
            # print("Ticks: ", pup.state.ticks, "Update Rate: ", filtered_control_rate)

            # Run the control loop
            observation = pup.get_observation()
            joystick_vals = joystick.joystick_values()
            if joystick_vals["L2"] < 0:
                behavior_state_override = "trot"
            else:
                behavior_state_override = "rest"

            com_x_shift += -1.0 * joystick_vals["d_pad_y"] * pup.config.dt / 100.0
            com_x_shift = min(max(com_x_shift, -0.05), 0.05)
            com_x_shift = DEFAULT_X_SHIFT
            height += (
                (-joystick_vals["x"] + joystick_vals["triangle"]
                 ) * pup.config.dt / 25.0
            )
            height = min(max(height, -0.25), -0.05)

            pup.step(
                action={
                    "x_velocity": joystick_vals["left_y"] / 2.0,#1.5,
                    "y_velocity": -joystick_vals["left_x"] / 4.0,#1.5,
                    "yaw_rate": joystick_vals["right_x"] * -3.0,#-4,
                    "pitch": joystick_vals["right_y"] * 0.5,
                    "height": height,
                    "com_x_shift": com_x_shift,
                },
                behavior_state_override=behavior_state_override,
            )
            # print("Foot coordinates: \n", pup.state.final_foot_locations)

    finally:
        pup.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--half", action="store_true", help="Enable flag half")
    args = parser.parse_args()
    run_example(half_robot=args.half)
