import numpy as np
import time
import rospy
import sys
from std_msgs.msg import Float64
import signal
import socket
import platform
from dingo_peripheral_interfacing.msg import ElectricalMeasurements


#Fetching is_sim and is_physical from arguments
args = rospy.myargv(argv=sys.argv)
if len(args) != 4: #arguments have not been provided, go to defaults (not sim, is physical)
    is_sim = 0
    is_physical = 1
    use_imu = 1
else:
    is_sim = int(args[1])
    is_physical = int(args[2])
    use_imu = int(args[3])

from dingo_control.Controller import Controller
from dingo_input_interfacing.InputInterface import InputInterface
from dingo_control.State import State
from dingo_control.Kinematics import four_legs_inverse_kinematics
from dingo_control.Config import Configuration

if is_physical:
    from dingo_servo_interfacing.HardwareInterface import HardwareInterface
    from dingo_peripheral_interfacing.IMU import IMU
    from dingo_control.Config import Leg_linkage

def signal_handler(sig, frame):
    sys.exit(0)

def main():
    """Main program
    """
    rospy.init_node("dingo_driver") 
    message_rate = 50
    rate = rospy.Rate(message_rate)

    signal.signal(signal.SIGINT, signal_handler)

    #TODO: Create a publisher for joint states 
    if is_sim:
        command_topics = ["/dingo_controller/FR_theta1/command",
                  "/dingo_controller/FR_theta2/command",
                  "/dingo_controller/FR_theta3/command",
                  "/dingo_controller/FL_theta1/command",
                  "/dingo_controller/FL_theta2/command",
                  "/dingo_controller/FL_theta3/command",
                  "/dingo_controller/RR_theta1/command",
                  "/dingo_controller/RR_theta2/command",
                  "/dingo_controller/RR_theta3/command",
                  "/dingo_controller/RL_theta1/command",
                  "/dingo_controller/RL_theta2/command",
                  "/dingo_controller/RL_theta3/command"]

        publishers = []
        for i in range(len(command_topics)):
            publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size = 0))

    # Create config
    config = Configuration()
    if is_physical:
        linkage = Leg_linkage(config)
        # hardware_interface = HardwareInterface(linkage)
        # Create imu handle
    if use_imu:
        imu = IMU()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )

    state = State()
    rospy.loginfo("Creating input listener...")
    input_interface = InputInterface(config)
    rospy.loginfo("Input listener successfully initialised... Robot will now receive commands via Joy messages")

    last_loop = time.time()

    rospy.loginfo("Summary of current gait parameters:")
    rospy.loginfo("overlap time: %.2f", config.overlap_time)
    rospy.loginfo("swing time: %.2f", config.swing_time)
    rospy.loginfo("z clearance: %.2f", config.z_clearance)
    rospy.loginfo("x shift: %.2f", config.x_shift)

    # Wait until the activate button has been pressed
    while not rospy.is_shutdown():
        rospy.loginfo("Waiting for L1 to activate robot.")
        while True:
            command = input_interface.get_command(state,message_rate)
            #input_interface\.set_color(config.ps4_deactivated_color)
            if command.joystick_control_event == 0:
                break
            rate.sleep()
        
        rospy.loginfo("Robot activated.")

        while True:
            #now = time.time()
            #if now - last_loop < config.dt:
            #    continue
            #last_loop = time.time()
            time.start = rospy.Time.now()
            # Parse the udp joystick commands and then update the robot controller's parameters
            command = input_interface.get_command(state,message_rate)
            if command.joystick_control_event == 1:
                rospy.loginfo("Deactivating Robot")
                break

            # Read imu data. Orientation will be None if no data was available
            # rospy.loginfo(imu.read_orientation())
            state.euler_orientation = (
                imu.read_orientation() if use_imu else np.array([0, 0, 0])
            )
            [yaw,pitch,roll] = state.euler_orientation
            #print('Yaw: ',np.round(yaw,2),'Pitch: ',np.round(pitch,2),'Roll: ',np.round(roll,2))
            # Step the controller forward by dt
            controller.run(state, command)

            #rospy.loginfo(state.joint_angles)
            # rospy.loginfo('State.height: ', state.height)

            #TODO here: publish the joint values (in state.joint_angles) to a publisher
            #If running simulator, publish joint angles to gazebo controller:
            if is_sim:
                rows, cols = state.joint_angles.shape
                i = 0
                for col in range(cols):
                    for row in range(rows):
                        publishers[i].publish(state.joint_angles[row,col])
                        i = i + 1
            
            # if is_physical:
            #     # Update the pwm widths going to the servos
            #     hardware_interface.set_actuator_postions(state.joint_angles)
            
            # rospy.loginfo('All angles: \n',np.round(np.degrees(state.joint_angles),2))
            time.end = rospy.Time.now()
            #Uncomment following line if want to see how long it takes to execute a control iteration
            #rospy.loginfo(str(time.start-time.end))

            # rospy.loginfo('State: \n',state)
            rate.sleep()

        #input_interface.set_color(config.ps4_color)
main()
