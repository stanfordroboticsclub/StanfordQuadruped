import pigpio
from src.HardwareInterface import initialize_pwm, send_servo_command
from src.PupperConfig import PWMParams, ServoParams
import numpy as np


def get_motor_name(i, j):
    motor_type = {0: "Abduction", 1: "Inner", 2: "Outer"}  # Top  # Bottom
    leg_pos = {0: "Front Right", 1: "Front Left", 2: "Back Right", 3: "Back Left"}
    final_name = motor_type[i] + " " + leg_pos[j]
    return final_name


def get_motor_setpoint(i, j):
    data = [[0, 0, 0, 0], [45, 45, 45, 45], [45, 45, 45, 45]]
    return data[i][j]


def degrees_to_radians(input_array):
    """Converts degrees to radians.
    
    Parameters
    ----------
    input_array :  Numpy array or float
        Degrees
    
    Returns
    -------
    Numpy array or float
        Radians
    """
    return np.pi / 180.0 * input_array


def step_until(servo_params, pi_board, pwm_params, kValue, i_index, j_index, set_point):
    """Returns the angle offset needed to correct a given link by asking the user for input.

    Returns
    -------
    Float
        Angle offset needed to correct the link.
    """
    found_position = False
    set_names = ["horizontal", "horizontal", "vertical"]
    offset = 0
    while not found_position:
        above_or_below = str(
            input(
                "Desired position: "
                + set_names[i_index]
                + ". Enter 'a' or 'b' to raise or lower the link. Enter 'd' when done. Input: "
            )
        )
        if above_or_below == "above" or above_or_below == "a":
            offset += 1.0
            send_servo_command(
                pi_board,
                pwm_params,
                servo_params,
                degrees_to_radians(set_point + offset),
                i_index,
                j_index,
            )
        elif above_or_below == "below" or above_or_below == "b":
            offset -= 1.0
            send_servo_command(
                pi_board,
                pwm_params,
                servo_params,
                degrees_to_radians(set_point + offset),
                i_index,
                j_index,
            )
        elif above_or_below == "done" or above_or_below == "d":
            found_position = True
        print("offset: ", offset, " original: ", set_point)

    return offset


def calibrate_b(servo_params, pi_board, pwm_params):
    """Calibrate the angle offset for the twelve motors on the robot. Note that servo_params is modified in-place.
    Parameters
    ----------
    servo_params : ServoParams
        Servo parameters. This variable is updated in-place.
    pi_board : Pi
        RaspberryPi object.
    pwm_params : PWMParams
        PWMParams object.
    """

    # Found K value of (11.4)
    kValue = float(
        input("Please provide a K value (microseconds per degree) for your servos: ")
    )
    servo_params.micros_per_rad = kValue * 180 / np.pi

    servo_params.neutral_angle_degrees = np.zeros((3, 4))

    for j in range(4):
        for i in range(3):
            # Loop until we're satisfied with the calibration
            completed = False
            while not completed:
                motor_name = get_motor_name(i, j)
                print("Currently calibrating " + motor_name + "...")
                set_point = get_motor_setpoint(i, j)

                # Move servo to set_point angle
                send_servo_command(
                    pi_board,
                    pwm_params,
                    servo_params,
                    degrees_to_radians(set_point),
                    i,
                    j,
                )

                # Adjust the angle using keyboard input until it matches the reference angle
                offset = step_until(
                    servo_params, pi_board, pwm_params, kValue, i, j, set_point
                )
                print("Final offset: ", offset)

                # The upper leg link has a different equation because we're calibrating to make it horizontal, not vertical
                if i == 1:
                    servo_params.neutral_angle_degrees[i, j] = set_point - offset
                else:
                    servo_params.neutral_angle_degrees[i, j] = -(set_point + offset)
                print("New beta angle: ", servo_params.neutral_angle_degrees[i, j])

                # Send the servo command using the new beta value and check that it's ok
                send_servo_command(
                    pi_board,
                    pwm_params,
                    servo_params,
                    degrees_to_radians([0, 45, -45][i]),
                    i,
                    j,
                )
                okay = ""
                while okay not in ["yes", "no"]:
                    okay = str(
                        input("Check angle. Are you satisfied? Enter 'yes' or 'no': ")
                    )
                completed = okay == "yes"


def main():
    """Main program
    """
    pi_board = pigpio.pi()
    pwm_params = PWMParams()
    servo_params = ServoParams()
    initialize_pwm(pi_board, pwm_params)

    calibrate_b(servo_params, pi_board, pwm_params)
    print("Calibrated neutral angles:")
    print(servo_params.neutral_angle_degrees)


main()
