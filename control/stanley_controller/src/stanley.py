#!/usr/bin/env python

import time ,math
import numpy as np
import math
import rospy
import copy
from sensor_msgs import msg
import tf
from aam_common_msgs.msg import Cone
from aam_common_msgs.msg import ConeDetections
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import random
from tf.transformations import euler_from_quaternion
from scipy.spatial import Delaunay
import sys
from sensor_msgs.msg import Imu
import bisect
from ackermann_msgs.msg import AckermannDriveStamped











class stanley_controller:
  
  def __init__(self):


    rospy.init_node("stanley_controller", anonymous = True)


    self.k = 0.5  # control gain
    self.Kp = 1.0  # speed proportional gain
    self.dt = 0.1  # [s] time difference
    self.L = 1.5  # [m] Wheel base of vehicle
    self.max_steer = np.radians(30.0)  # [rad] max steering angle
    self.ds = 0.1  # [m] distance of each interpolated points
    self.target_speed = 1.0  # [m/s]
    self.K = 0.78     # STANLEY GAIN:



    self.waypoints_x = []
    self.waypoints_y = []

    self.Vx = 0.0
    self.Vy = 0.0
    self.yaw = 0.0

    self.control_steering_angle = 0.0

    self.vel_cmd = 0.0
    self.steer_cmd = 0.0

    rospy.Subscriber('/visual/waypoints',MarkerArray,self.waypoints_callback)
    #rospy.Subscriber('/sensor_imu_hector',Imu,self.imu_callback)
    rospy.Subscriber("/ground_truth/state",Odometry,self.odom_callback)
    rospy.Subscriber("/robot_control/command",AckermannDriveStamped,self.control_callback)
    self.robot_control_pub = rospy.Publisher("/robot_control/command",AckermannDriveStamped,queue_size=0)






  def control_callback(self,control_msg):
    self.control_steering_angle = control_msg.drive.steering_angle
    self.control_velocity = control_msg.drive.speed
    print("ana f control callback")



  '''def imu_callback(self,imu_msg):
    self.yaw = imu_msg.orientation.z
    print("a7a")'''



  def odom_callback(self,odom_msg):
    self.Vx = odom_msg.twist.twist.linear.x
    self.Vy = odom_msg.twist.twist.linear.y
    print("odom callback")



  def waypoints_callback(self,waypoints_msg):
    print("waypoints callback")

    if len(waypoints_msg.markers[0].points)<2:
      return

    for point in waypoints_msg.markers[0].points:
      self.waypoints_x.append(point.x)
      self.waypoints_y.append(point.y)

    ### speed PID Controller ###
    self.vel_cmd = self.pid_control(self.target_speed,self.Vx)

    ### Stanley Steering Controller ###
    self.steer_cmd = self.stanley_control()

    ### publish control command ###
    print(self.vel_cmd)
    self.control_command(self.steer_cmd,self.vel_cmd)




  def stanley_control(self):


    # get equation of line between two waypoints y=m*x+b
    x1 = self.waypoints_x[0]
    y1 = self.waypoints_y[0]
    x2 = self.waypoints_x[len(self.waypoints_x)-1]
    y2 = self.waypoints_y[len(self.waypoints_x)-1]
    m , b = self.line_equation(x1,y1,x2,y2)

    # calculate cross track error
    e = b/(m-1)

    # calculate cross steer error
    es = math.atan2((self.K*e),self.Vx+0.0000001)

    # calculate heading error
    epsi = math.atan2(-m,-1) - self.control_steering_angle

    # calculate total steering input
    delta = epsi + es

    return delta




  def line_equation(self,x1,y1,x2,y2):

    dy = y2-y1
    dx = x2-x1

    m = dy/dx
    b = y2 - m*x2

    return m ,b




  def pid_control(self,target, current):
    """
    Proportional control for the speed.
    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return self.Kp * (target - current)
    



  def control_command(self,steering,velocity):
    control_msg = AckermannDriveStamped()

    control_msg.drive.steering_angle = steering
    control_msg.drive.speed = velocity
    
    self.robot_control_pub.publish(control_msg)











if __name__ == '__main__':
  try:
      control = stanley_controller()
  except rospy.ROSInterruptException:
      pass
  rospy.spin()
