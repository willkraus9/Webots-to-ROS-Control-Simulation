#! /usr/bin/env python3

import numpy as np
import math
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion 
from geometry_msgs.msg import Twist

# Utility Functions
def wrapToPi(a):
    # Wraps the input angle to between 0 and pi
    return (a + np.pi) % (2 * np.pi) - np.pi

def clamp(n, minValue, maxValue):
    # Clamps to range [minValue, maxValue]
    return max(min(maxValue, n), minValue)

def closestNode(X, Y, trajectory):
    # Find the closest waypoints in the trajectory with respect to the point [X, Y] given as input
    point = np.array([X, Y])
    trajectory = np.asarray(trajectory)
    dist = point - trajectory
    distSquared = np.sum(dist**2, axis = 1)
    minIndex = np.argmin(distSquared)
    return np.sqrt(distSquared[minIndex]), minIndex

def getTrajectory(filename):
        # Import .csv file and stores the waypoints in a 2D numpy array and then into trajectory
        with open(filename) as f:
            lines = f.readlines()
            trajectory = np.zeros((len(lines), 2))
            for idx, line in enumerate(lines):
                x = line.split(",")
                trajectory[idx, 0] = x[0]
                trajectory[idx, 1] = x[1]
        return trajectory
 
# Algorithm 1: PID
# 1. import "odometry/filtered" --> linear quaternion (X,Y,Z,psi) 
# 2. process .csv course 
# 3. process PID 
# 4. export values (psi, X_dot = 1 m/s) --> cmd_vel geometry_msg/Twist

class CMU_PID():
    
    def __init__(self):
        # .csv course variable
        self.trajectory = 0
          
        # PID control variables
        self.error_ki = 0
        self.last_error_kp = 0
        self.error_ki_F = 0
        self.last_error_kp_F = 0
        
        # Odometry variables
        self.X_gps = 0
        self.Y_gps = 0
        self.psi_gps = 0
        
        # Subscriber and Publisher 
        self.sub = rospy.Subscriber("/odometry/filtered", Odometry, self.update)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)     
        
    def update(self, msg):
        trajectory = self.trajectory
        error_ki = self.error_ki
        last_error_kp = self.last_error_kp  
        
        # Import state of robot from /odometry/filtered 
        X_gps = msg.pose.pose.position.x
        Y_gps = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (phi_gps, theta_gps, psi_gps) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w]) 

         # Part of an assignment, reverted back to original format
        
        #TODO: get the node that the vehicle tracks from the .csv file using built-in functions 
            #you will have to change the path that buggyTrace.csv is stored

        # TODO: make a PID controller for the target psi value and F value using the odometry topic from ROS
        # Export delta and F values to cmd_vel geometry_msg/Twist
        heading = Twist()
        print("Delta: ",delta)
        print("F: ", F)
        # clamp F to be between 0 and 1 due to simulation environment / model
        F = max(0, F*(0.01))
        print("============")

        # NB: 0.25 in gazebo = pi/2 in radians
        # make a 90 degree turn
        #if psi_gps <= 0.25:
        #    heading.angular.z = 0.25
        #else:
        #    heading.linear.x = 0
        #    heading.angular.z = 0
        #    print("stop")
        #    rospy.sleep(1000)

        # Part of an assignment, reverted back to original format
        
        #TODO: get the node that the vehicle tracks from the .csv file using built-in functions 
            #you will have to change the path that buggyTrace.csv is stored

        # TODO: make a PID controller for the target psi value using the odometry topic from ROS
        
        # Export delta and F values to cmd_vel geometry_msg/Twist
        heading = Twist()
        
        #DEBUGGING
        print ("X_GPS: ", X_gps)
        print("Y_GPS: ", Y_gps)
        print ("Psi_raw: ",psi_gps)
        print ("Quaternion: ")
        print(rot_q)
        print("Delta: ",delta)
        print("============")
        
        if abs(psi_gps*2*np.pi - delta) <= 0.1:
            heading.linear.x = F
            rospy.sleep(0.25)
            print("FORWARD")
        else:
            heading.linear.x = 0
            heading.angular.z = math.copysign(0.25,delta)
            
        r = rospy.Rate(10) #10 Hz publishing rate            
        self.pub.publish(heading)
        r.sleep()

if __name__ == '__main__':
    rospy.init_node('CMU_PID_CONTROL')
    CMU_PID() 
    rospy.spin()
