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
from dingo_control.State import State, BehaviorState
from dingo_control.Kinematics import four_legs_inverse_kinematics
from dingo_control.Config import Configuration
from dingo_control.msg import TaskSpace, JointSpace, Angle
from std_msgs.msg import Bool

if is_physical:
    from dingo_servo_interfacing.HardwareInterface import HardwareInterface
    from dingo_peripheral_interfacing.IMU import IMU
    from dingo_control.Config import Leg_linkage

class DingoDriver:
    def __init__(self,is_sim, is_physical, use_imu):
        self.message_rate = 50
        self.rate = rospy.Rate(self.message_rate)

        self.is_sim = is_sim
        self.is_physical = is_physical
        self.use_imu = use_imu

        self.joint_command_sub = rospy.Subscriber("/joint_space_cmd", JointSpace, self.run_joint_space_command)
        self.task_command_sub = rospy.Subscriber("/task_space_cmd", TaskSpace, self.run_task_space_command)
        self.estop_status_sub = rospy.Subscriber("/emergency_stop_status", Bool, self.update_emergency_stop_status)
        self.external_commands_enabled = 0

        if self.is_sim:
            self.sim_command_topics = ["/dingo_controller/FR_theta1/command",
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

            self.sim_publisher_array = []
            for i in range(len(self.sim_command_topics)):
                self.sim_publisher_array.append(rospy.Publisher(self.sim_command_topics[i], Float64, queue_size = 0))

        # Create config
        self.config = Configuration()
        if is_physical:
            self.linkage = Leg_linkage(self.config)
            self.hardware_interface = HardwareInterface(self.linkage)
            # Create imu handle
        if self.use_imu:
            self.imu = IMU()

        # Create controller and user input handles
        self.controller = Controller(
            self.config,
            four_legs_inverse_kinematics,
        )

        self.state = State()
        rospy.loginfo("Creating input listener...")
        self.input_interface = InputInterface(self.config)
        rospy.loginfo("Input listener successfully initialised... Robot will now receive commands via Joy messages")

        rospy.loginfo("Summary of current gait parameters:")
        rospy.loginfo("overlap time: %.2f", self.config.overlap_time)
        rospy.loginfo("swing time: %.2f", self.config.swing_time)
        rospy.loginfo("z clearance: %.2f", self.config.z_clearance)
        rospy.loginfo("back leg x shift: %.2f", self.config.rear_leg_x_shift)
        rospy.loginfo("front leg x shift: %.2f", self.config.front_leg_x_shift)

        
    
    def run(self):
        # Wait until the activate button has been pressed
        while not rospy.is_shutdown():
            if self.state.currently_estopped == 1:
                rospy.logwarn("E-stop pressed. Controlling code now disabled until E-stop is released")
                self.state.trotting_active = 0
                while self.state.currently_estopped == 1:
                    self.rate.sleep()
                rospy.loginfo("E-stop released")
            
            rospy.loginfo("Manual robot control active. Currently not accepting external commands")
            #Always start Manual control with the robot standing still. Send default positions once
            command = self.input_interface.get_command(self.state,self.message_rate)
            self.state.behavior_state = BehaviorState.REST
            self.controller.run(self.state, command)
            self.controller.publish_joint_space_command(self.state.joint_angles)
            self.controller.publish_task_space_command(self.state.rotated_foot_locations)
            if self.is_sim:
                    self.publish_joints_to_sim(self.state.joint_angles)
            if self.is_physical:
                # Update the pwm widths going to the servos
                self.hardware_interface.set_actuator_postions(self.state.joint_angles)
            while self.state.currently_estopped == 0:
                time.start = rospy.Time.now()

                #Update the robot controller's parameters
                command = self.input_interface.get_command(self.state,self.message_rate)
                if command.joystick_control_event == 1:
                    if self.state.currently_estopped == 0:
                        self.external_commands_enabled = 1
                        break
                    else:
                        rospy.logerr("Received Request to enable external control, but e-stop is pressed so the request has been ignored. Please release e-stop and try again")
                
                # Read imu data. Orientation will be None if no data was available
                # rospy.loginfo(imu.read_orientation())
                self.state.euler_orientation = (
                    self.imu.read_orientation() if self.use_imu else np.array([0, 0, 0])
                )
                [yaw,pitch,roll] = self.state.euler_orientation
                # print('Yaw: ',np.round(yaw,2),'Pitch: ',np.round(pitch,2),'Roll: ',np.round(roll,2))
                # Step the controller forward by dt
                self.controller.run(self.state, command)

                if self.state.behavior_state == BehaviorState.TROT: #If trotting
                    self.controller.publish_joint_space_command(self.state.joint_angles)
                    self.controller.publish_task_space_command(self.state.rotated_foot_locations)
                    #rospy.loginfo(state.joint_angles)
                    # rospy.loginfo('State.height: ', state.height)

                    #If running simulator, publish joint angles to gazebo controller:
                    if self.is_sim:
                        self.publish_joints_to_sim(self.state.joint_angles)
                    if self.is_physical:
                        # Update the pwm widths going to the servos
                        self.hardware_interface.set_actuator_postions(self.state.joint_angles)
                    
                    # rospy.loginfo('All angles: \n',np.round(np.degrees(state.joint_angles),2))
                    time.end = rospy.Time.now()
                    #Uncomment following line if want to see how long it takes to execute a control iteration
                    #rospy.loginfo(str(time.start-time.end))

                    # rospy.loginfo('State: \n',state)
                self.rate.sleep()

            if self.state.currently_estopped == 0:
                rospy.loginfo("Manual Control deactivated. Now accepting external commands")
                command = self.input_interface.get_command(self.state,self.message_rate)
                self.state.behavior_state = BehaviorState.REST
                self.controller.run(self.state, command)
                self.controller.publish_joint_space_command(self.state.joint_angles)
                self.controller.publish_task_space_command(self.state.rotated_foot_locations)
                if self.is_sim:
                        self.publish_joints_to_sim(self.state.joint_angles)
                if self.is_physical:
                    # Update the pwm widths going to the servos
                    self.hardware_interface.set_actuator_postions(self.state.joint_angles)
                while self.state.currently_estopped == 0:
                    command = self.input_interface.get_command(self.state,self.message_rate)
                    if command.joystick_control_event == 1:
                        self.external_commands_enabled = 0
                        break
                    self.rate.sleep()
    
    def update_emergency_stop_status(self, msg):
        if msg == 1:
            self.state.currently_estopped = 1
        if msg == 0:
            self.state.currently_estopped = 0
        return

    def run_task_space_command(self, msg):
        if self.external_commands_enabled == 1 and self.currently_estopped == 0:
            foot_locations = np.zeros((3,4))
            j = 0
            for i in 3:
                foot_locations[i] = [msg.FR_foot[j], msg.FL_foot[j], msg.RR_foot[j], msg.RL_foot[j]]
                j = j+1
            print(foot_locations)
            joint_angles = self.controller.inverse_kinematics(foot_locations, self.config)
            if self.is_sim:
                self.publish_joints_to_sim(self, joint_angles)
            
            if self.is_physical:
                self.hardware_interface.set_actuator_postions(joint_angles)
            
        elif self.external_commands_enabled == 0:
            rospy.logerr("ERROR: Robot not accepting commands. Please deactivate manual control before sending control commands")
        elif self.currently_estopped == 1:
            rospy.logerr("ERROR: Robot currently estopped. Please release before trying to send commands")

    def run_joint_space_command(self, msg):
        if self.external_commands_enabled == 1 and self.currently_estopped == 0:
            joint_angles = np.zeros((3,4))
            j = 0
            for i in 3:
                joint_angles[i] = [msg.FR_foot[j], msg.FL_foot[j], msg.RR_foot[j], msg.RL_foot[j]]
                j = j+1
            print(joint_angles)

            if self.is_sim:
                self.publish_joints_to_sim(self, joint_angles)
            
            if self.is_physical:
                self.hardware_interface.set_actuator_postions(joint_angles)
            
        elif self.external_commands_enabled == 0:
            rospy.logerr("ERROR: Robot not accepting commands. Please deactivate manual control before sending control commands")
        elif self.currently_estopped == 1:
            rospy.logerr("ERROR: Robot currently estopped. Please release before trying to send commands")
    
    def publish_joints_to_sim(self, joint_angles):
        rows, cols = joint_angles.shape
        i = 0
        for col in range(cols):
            for row in range(rows):
                self.sim_publisher_array[i].publish(joint_angles[row,col])
                i = i + 1



def signal_handler(sig, frame):
    sys.exit(0)

def main():
    """Main program
    """
    rospy.init_node("dingo_driver") 
    signal.signal(signal.SIGINT, signal_handler)
    dingo = DingoDriver(is_sim, is_physical, use_imu)
    dingo.run()
    
main()
