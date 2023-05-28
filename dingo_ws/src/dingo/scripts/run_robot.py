import numpy as np
import time
import rospy
import sys
from std_msgs.msg import Float64
import signal
import socket
import platform
from dingo_peripheral_interfacing.msg import ElectricalMeasurements

#Module for i2c testing
import subprocess

#Fetching is_sim and is_physical from arguments
args = rospy.myargv(argv=sys.argv)
if len(args) != 3: #arguments have not been provided, go to defaults (not sim, is physical)
    is_sim = 0
    is_physical = 0
else:
    is_sim = int(args[1])
    is_physical = int(args[2])

from dingo_peripheral_interfacing.IMU import IMU
from dingo_control.Controller import Controller
from dingo_input_interfacing.InputInterface import InputInterface
from dingo_control.State import State
from dingo_control.Kinematics import four_legs_inverse_kinematics
from dingo_control.Config import Configuration
from dingo_input_interfacing.InputController import InputController

if is_physical:
    from dingo_servo_interfacing.HardwareInterface import HardwareInterface
    from dingo_control.Config import Leg_linkage

def signal_handler(sig, frame):
    sys.exit(0)

def main(use_imu=False):
    """Main program
    """
    rospy.init_node("dingo") 
    message_rate = 50
    rate = rospy.Rate(message_rate)

    signal.signal(signal.SIGINT, signal_handler)

    #TODO: Create a publisher for joint states 
    if is_sim:
        command_topics = ["/notspot_controller/FR1_joint/command",
                  "/notspot_controller/FR2_joint/command",
                  "/notspot_controller/FR3_joint/command",
                  "/notspot_controller/FL1_joint/command",
                  "/notspot_controller/FL2_joint/command",
                  "/notspot_controller/FL3_joint/command",
                  "/notspot_controller/RR1_joint/command",
                  "/notspot_controller/RR2_joint/command",
                  "/notspot_controller/RR3_joint/command",
                  "/notspot_controller/RL1_joint/command",
                  "/notspot_controller/RL2_joint/command",
                  "/notspot_controller/RL3_joint/command"]

        publishers = []
        for i in range(len(command_topics)):
            publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size = 0))

    # Create config
    config = Configuration()
    if is_physical:
        linkage = Leg_linkage(config)
        hardware_interface = HardwareInterface(linkage)
        # Create imu handle
        if use_imu:
            imu = IMU(port="/dev/ttyACM0")
            imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()
    print("Creating input listener...")
    input_interface = InputInterface(config)
    print(platform.processor())
    input_Controller = InputController(1, platform.processor()) #Check raspi hostname and replace here
    print("Done.")

    last_loop = time.time()

    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)

    # Wait until the activate button has been pressed

    loop = 0
    while not rospy.is_shutdown():      
        print("Waiting for L1 to activate robot.")
        
        while True:
            command = input_interface.get_command(state,message_rate)
            # hardware_interface.relax_all_motors()
            #input_interface\.set_color(config.ps4_deactivated_color)
            if command.joystick_control_event == 1:
                break
            rate.sleep()
        print("Robot activated.")
        
        #input_interface.set_color(config.ps4_color)

        while True:
            #now = time.time()
            #if now - last_loop < config.dt:
            #    continue
            #last_loop = time.time()
            time.start = rospy.Time.now()
            # Parse the udp joystick commands and then update the robot controller's parameters
            command = input_interface.get_command(state,message_rate)
            if command.joystick_control_event == 1:
                print("Deactivating Robot")
                break

            # Read imu data. Orientation will be None if no data was available
            quat_orientation = (
                imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
            )
            state.quat_orientation = quat_orientation

            # Step the controller forward by dt
            controller.run(state, command)
            # print('State.height: ', state.height)

            #TODO here: publish the joint values (in state.joint_angles) to a publisher
            #If running simulator, publish joint angles to gazebo controller:
            if is_sim:
                rows, cols = state.joint_angles.shape
                print(rows)
                print(cols)
                for row in rows:
                    for col in cols:
                        publishers[rows*row+col].publish(state.joint_angles[rows*row+col])
            
            if is_physical:
                # Update the pwm widths going to the servos
                
                hardware_interface.set_actuator_postions(state.joint_angles)
            
            # print('All angles: \n',np.round(np.degrees(state.joint_angles),2))
            # print('All angles: \n',np.round(np.degrees(state.joint_angles[:,0] - state.joint_angles[:,3]),2))
            time.end = rospy.Time.now()
            #Uncomment following line if want to see how long it takes to execute a control iteration
            #print(str(time.start-time.end))
            loop +=1
            # print('Iteration: ',loop)
            # print('State: \n',state)
            rate.sleep()

main()
