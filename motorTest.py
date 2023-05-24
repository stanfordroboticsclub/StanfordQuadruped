from adafruit_servokit import ServoKit
# from basic_kinematics import *
import numpy as np
import math as m

import time 

from servoDefinition import motor_config

Dingo  = motor_config()
Dingo.relax_all_motors()
# Dingo.moveAbsAngle(Dingo.back_left_hip, 88)
# Dingo.moveAbsAngle(Dingo.front_left_hip,105)
# Dingo.moveAbsAngle(Dingo.back_left_upper,11)
# Dingo.moveAbsAngle(Dingo.back_left_lower,14+90)

# Dingo.calibrate_servo(Dingo.front_left_upper)


#-------- MOVING CALIBRATED LEGS TO THE HOME POSITOIN -------- #
# ## HOme positoin values: 
home = [0,0,90]
low = [0,25,140]
mid = [0,42,120]
high = [0,50,110] #hip, upper leg, lower leg [0,42,125]    #[29, 17, 22, 9],
pos = home

""" SERVO INDICES, CALIBRATION MULTIPLIERS AND OFFSETS
            #   ROW:    which joint of leg to control 0:hip, 1: upper leg, 2: lower leg
            #   COLUMN: which leg to control. 0: front-right, 1: front-left, 2: back-right, 3: back-left.

                #               0                  1                2               3
                #  0 [[front_right_hip  , front_left_hip  , back_right_hip  , back_left_hip  ]
                #  1  [front_right_upper, front_left_upper, back_right_upper, back_left_upper]
                #  2  [front_right_lower, front_left_lower, back_right_lower, back_left_lower]] """

offsets = np.array(
                    [[77, 81, 115, 76], 
                    [29, 8, 33, 13], 
                    [26, 13, 30, 4]])

                    

                    # [[83, 79, 108, 79], 
                    # [28, 13, 28, 19], 
                    # [22, 16, 24, 6]])

Dingo.moveAbsAngle(Dingo.front_right_hip  ,offsets[0,0]+pos[0])
Dingo.moveAbsAngle(Dingo.front_right_upper,offsets[1,0]+pos[1])
Dingo.moveAbsAngle(Dingo.front_right_lower,offsets[2,0]+pos[2])

Dingo.moveAbsAngle(Dingo.front_left_hip  ,offsets[0,1]+pos[0])
Dingo.moveAbsAngle(Dingo.front_left_upper,offsets[1,1]+pos[1])
Dingo.moveAbsAngle(Dingo.front_left_lower,offsets[2,1]+pos[2])

Dingo.moveAbsAngle(Dingo.back_right_hip  ,offsets[0,2]+pos[0])
Dingo.moveAbsAngle(Dingo.back_right_upper,offsets[1,2]+pos[1])
Dingo.moveAbsAngle(Dingo.back_right_lower,offsets[2,2]+pos[2])

Dingo.moveAbsAngle(Dingo.back_left_hip  , offsets[0,3]+pos[0])
Dingo.moveAbsAngle(Dingo.back_left_upper,offsets[1,3]+pos[1])
Dingo.moveAbsAngle(Dingo.back_left_lower,offsets[2,3]+pos[2])


# def moveAllLegs(pos):
#     print('tada')
    #Already Calubrated

# start_pos = np.array(low)
# end_pos = np.array(high)
# start_time = time.time()
# total_time = 0
# moveAllLegs(start_pos)
# cycle_time = 10
# dt = 0.1
# pos = start_pos
# print('Pos:' ,pos)
# print(np.all(pos<end_pos))
# while pos[1]<end_pos[1] and pos[2]>end_pos[2]:
    
#     if round((time.time()-start_time),1)%dt < 0.3: #every 0.1 seconds
#         pos = pos + (end_pos-start_pos) * (dt/cycle_time)
#         print(np.round(pos))
#         moveAllLegs(np.round(pos,0))


# _________________________________________________ STRAIGH LINE TESTING

# class leg_linkage:
#     def __init__(self):
#         self.a = 35.12 #mm
#         self.b = 37.6 #mm
#         self.c = 45 #mm
#         self.d = 35.23  #mm
#         self.e = 67.1 #mm
#         self.f = 142.2 #mm
#         self.g = 37.07 #mm
#         self.h = 45 #mm
#         self.upper_leg_length = 142.25 #euivalent to i in the diagram
#         self.lower_leg_length = 162
#         self.i = self.upper_leg_length
#         self.hip_width = 50
#         self.gamma = m.atan(28.80/20.20)
#         self.EDC = m.acos((self.c**2+self.h**2-self.e**2)/(2*self.c*self.h))



# points = 25
# link = leg_linkage()
# x_traj = np.linspace(200,-200,points)
# y_traj = np.linspace(-220,-220,points)
# z_traj = np.linspace(link.hip_width,link.hip_width,points)

# # [THETA0, THETA1 ,THETA2] = follow_2Dtrajectory(link,x_traj,y_traj,z_traj,points)

# # kit.servo[0].angle = 0
# # kit.servo[1].angle = 0


# for i in range(points):
#     D = get_domain(link, x_traj[i], y_traj[i],z_traj[i])
#     THETA1,THETA2,THETA3 =  RightIK(link,x_traj[i], y_traj[i],z_traj[i],  D) #legIK(link,x_traj[i], y_traj[i], 0) 


    
    
#     #Calculating new link location coordinates for plotting
#     link1_angle = 3/2*m.pi + THETA2


#     # THETA2 = m.pi/2- THETA2

#     THETA0=th3_to_th0(link, THETA2, -THETA3)

#     THETA0 = m.pi/2 + m.pi-THETA0

#     THETA2 = m.pi/2- THETA2

#     # print('Theta1: ',m.degrees(THETA1))
#     # print('Theta0: ',m.degrees(THETA0))
#     # print('Theta2: ',m.degrees(THETA2))

#     pos = [0,round(m.degrees(THETA2),1),round(m.degrees(THETA0),1)]
#     print('Pos: ', pos)
#     # pos = mid
#     # print('Theta3: ',m.degrees(THETA3))


#     # kit.servo[0].angle = m.degrees(THETA2)
#     # kit.servo[1].angle = m.degrees(THETA0)

#     Dingo.moveAbsAngle(Dingo.back_right_hip,105+pos[0])
#     Dingo.moveAbsAngle(Dingo.back_right_upper,22+pos[1])
#     Dingo.moveAbsAngle(Dingo.back_right_lower,30+pos[2])

#     Dingo.moveAbsAngle(Dingo.front_right_hip,88+pos[0])
#     Dingo.moveAbsAngle(Dingo.front_right_upper,29+pos[1])
#     Dingo.moveAbsAngle(Dingo.front_right_lower,24+pos[2])

#     Dingo.moveAbsAngle(Dingo.front_left_hip,78+pos[0])
#     Dingo.moveAbsAngle(Dingo.front_left_upper,17+pos[1])
#     Dingo.moveAbsAngle(Dingo.front_left_lower,0+pos[2])

#     Dingo.moveAbsAngle(Dingo.back_left_hip, 83+pos[0])
#     Dingo.moveAbsAngle(Dingo.back_left_upper,9+pos[1])
#     Dingo.moveAbsAngle(Dingo.back_left_lower,5+pos[2])


#     time.sleep(0.1)
