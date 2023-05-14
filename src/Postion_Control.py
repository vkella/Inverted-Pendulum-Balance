#!/usr/bin/env python3
from control_msgs.msg import JointControllerState as JCS
from sensor_msgs.msg import JointState as JS
from std_msgs.msg import Float64
from dynamic_reconfigure.msg import Config, DoubleParameter
import rospy
import math 
import std_msgs.msg
import numpy as np
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Point
import dynamic_reconfigure.client
from math import atan2
cart_pose = np.array([0., 0., 0., 0.])
d=1
##### Callback to update the theta and theta_dot values of Pole     
def takeangle(angle_message):
    cart_pose[2] = angle_message.process_value
    cart_pose[3] = angle_message.process_value_dot
    
##### Callback to update the Postion and Velocity values of Cart Pole     
def takepose(pose_message):
    cart_pose[0] = pose_message.position[1]
    cart_pose[1] = pose_message.velocity[1]
     
####### Function to Move the Cart and Pole ##########################
def movePole(d,theta,a,b):
    rospy.init_node('Move_Pole')
    cart_publisher=rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=50)
    pole_publisher=rospy.Publisher('/invpend/joint2_position_controller/command', Float64, queue_size=50)
    vel_msg  = Float64()
    #vel_msg.data=velocity
    if(cart_pose[0] < d):
        while(cart_pose[0] < d):
            vel_msg.data=0.5
            cart_publisher.publish(vel_msg)
    else:
        while(cart_pose[0]>d):
            vel_msg.data=-0.5
            cart_publisher.publish(vel_msg)
    print("Pose Endeffector Locations:(",a," ",b,")")
    print("Cart Postion:(",d," 0)")
    
    vel_msg.data=0.0
    cart_publisher.publish(vel_msg)
    pole_msg  = Float64()
    pole_msg.data=theta 
    pole_publisher.publish(pole_msg)

    
if __name__ == '__main__':
    theta_sub = rospy.Subscriber("/invpend/joint2_position_controller/state",JCS, takeangle)
    pos_sub = rospy.Subscriber("/invpend/joint_states",JS, takepose)
    ######### Equation to find the postion of Cart & Angle of pole based on the x, y points (waypoints) ####################
    
    ### Equation of Circle, To Find the center given the a point on a the circle.
    #Assuming that the pole is placed on the Center of cart and assume pole to be a radius of circle and we need to calculate the center and Arc angle w.r.t y axis.
    h=lambda x,y: x- np.sqrt(0.5**2 -y**2)
    # X coordinates of Point
    x=np.array([0.0,0.5,0.75,1.0,1.25,1.5,2.0])
    # Y coordinates of Point
    y=np.array([0.5,0.4,0.35,0.3,0.25,0.4,0.5]) 
    k=np.zeros(7)
    theta=np.zeros(7)
    for i in range(7):
        #Calculating the postion the cart need to moved.
        k[i]=h(x[i],y[i])
        # Calculating the angle the pole need to rotate to reach the point after the cart is at the respective position.
        theta[i]=1.5708-atan2(y[i],x[i]-k[i]) 
        movePole(k[i],theta[i],x[i],y[i])
        

    
    
        
