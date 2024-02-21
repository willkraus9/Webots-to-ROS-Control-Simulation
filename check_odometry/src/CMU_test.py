#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion #needs pip install tf?
from geometry_msgs.msg import Twist

class CMU_PID():
    
    def __init__(self):

        # Odometry variables
        self.X_gps = 0
        self.Y_gps = 0
        self.psi_gps = 0
        
        # Subscriber and Publisher 
        self.sub = rospy.Subscriber("/odometry/filtered", Odometry, self.update)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)     
        
    def update(self, msg):
        
        # Import state of robot from /odometry/filtered 
        self.X_gps = msg.pose.pose.position.x
        self.Y_gps = msg.pose.pose.position.y
        self.rot_q = msg.pose.pose.orientation
        (phi_gps, theta_gps, psi_gps) = euler_from_quaternion([self.rot_q.x, self.rot_q.y, self.rot_q.z, self.rot_q.w]) 
             
        # Export delta and F values to cmd_vel geometry_msg/Twist
        heading = Twist()
        r = rospy.Rate(10) #10 Hz publishing rate            
        if self.X_gps > 3:
            heading.linear.x = 0.3 # Velocity is capped at ~1 m/s
            heading.angular.z = 0.1
        else:
            heading.linear.x = 0.9
            heading.angular.z = -0.3
        self.pub.publish(heading)
        r.sleep()
    
# TODO: 
    # 1. put this into virtual machine (make it chmod, top line)
    # 2. modify .launch file from last time (make it the same as that one; rename this file!!!)
    #   2.5: run everything (and pray to the Omnissiah), revise for errors!
        # - pathing for files / edit privileges
        # - something with the libraries
        # - class structure 
        # - subscriber / publisher syntax 
        # - twist publishing layout
        # - internal issue with Jackal not accepting twists (although it should!)
    # 3. modify / test command line printing, cross-track error distance (something from LQR)

if __name__ == '__main__':
    rospy.init_node('CMU_PID_CONTROL')
    CMU_PID() 
    rospy.spin()
