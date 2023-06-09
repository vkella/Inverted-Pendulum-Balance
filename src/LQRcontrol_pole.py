#!/usr/bin/env python3
import rospy
import numpy as np
from control import lqr
from control_msgs.msg import JointControllerState as JCS
from sensor_msgs.msg import JointState as JS
from std_msgs.msg import Float64

############ A ,B,Q,R Matrix|A & B dervied Based on the Cart Mass, Pole mass,etc #####################
###########  X_dot = Ax+Bu #########################
A = np.matrix([ [0., 1.,0., 0.],
                [0., 0., -0.97190083, 0],
                [0., 0.,  0., 1.],
                [0., 0., 19.74961832, 0.]
                ])    

B = np.matrix([ [0.],
              [0.04961832],
               [0.],
               [-0.09160305]])

Q = np.matrix([[100.,0.,0.,0.],
	       [0.,100.,0.,0.],
               [0.,0.,100.,0.],
               [0.,0.,0.,100.]])

R = np.diag( [1.])

############## Finding the State feedback gains ######################
K, S, E = lqr( A, B, Q, R )
############Postion & Angle of Cart #######################################
cart_pose =np.array([0., 0., 0., 0.])  # General Format[x,x_dot,Θ,Θ_dot]
final_pose = np.array([0., 0., 0., 0.])


####################Balcing the Ploe using LQR , u=-Kx ######################
def pole_blanace():
    rospy.init_node('p_blanace')
    print("Cart Pole Pose & Angle",cart_pose)
    vel_publisher = rospy.Publisher("/invpend/joint1_velocity_controller/command", Float64, queue_size=50)
    vel_msg  = Float64()
    x=cart_pose-final_pose
    vel_msg.data = -np.matmul(K,x)     #u =-Kx
    vel_publisher.publish(vel_msg)
    
##### Callback to update the theta and theta_dot values of Pole     
def takeangle(angle_message):
    cart_pose[2] = angle_message.process_value
    cart_pose[3] = angle_message.process_value_dot
    
##### Callback to update the Postion and Velocity values of Cart Pole     
def takepose(pose_message):
    cart_pose[0] = pose_message.position[1]
    cart_pose[1] = pose_message.velocity[1]
    
    
if __name__ == '__main__':
    theta_sub = rospy.Subscriber("/invpend/joint2_position_controller/state",JCS, takeangle)
    pos_sub = rospy.Subscriber("/invpend/joint_states",JS, takepose) 
    while not rospy.is_shutdown():
        pole_blanace()
