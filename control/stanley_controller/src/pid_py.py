#!/usr/bin/env python

from cmath import atan, sin
import math
import numpy as np
import rospy
from sensor_msgs import msg
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from ackermann_msgs.msg import AckermannDriveStamped


class pid_controller:
    def __init__(self):

        rospy.init_node("pid_controller", anonymous = True)
        
        self.Kp = 0.13
        self.Ki = 0.02
        self.Kd = 0.001
        self.dt = 0.1

        self.d_error=0.0
        self.i_error=0.0
        self.p_error=0.0
        self.target_sp=0.0

        self.L = 1.58  # [m] Wheel base of vehicle
        #self.K = 0.78     # STANLEY GAIN:
        
        self.waypoints_x = []
        self.waypoints_y = []

        self.Vx = 0.0
        self.Vy = 0.0
        self.yaw = 0.0

        self.control_steering_angle = 0.0
        self.control_velocity = 0.0

        self.vel_cmd = 0.0
        
        self.steer_cmd = 0.0

        self.x = 0.0
        self.y = 0.0
        self.alpha = 0.0
        self.ld = 0.0
        self.look_ahead = 0.0
        self.gain = 1

        rospy.Subscriber('/visual/waypoints',MarkerArray,self.waypoints_callback)
        rospy.Subscriber('/sensor_imu_hector',Imu,self.imu_callback)
        rospy.Subscriber("/ground_truth/state",Odometry,self.odom_callback)
        #rospy.Subscriber("/robot_control/command",AckermannDriveStamped,self.control_callback)
        self.robot_control_pub = rospy.Publisher("/robot_control/command",AckermannDriveStamped,queue_size=0)

    '''def control_callback(self,control_msg):
        self.control_steering_angle = control_msg.drive.steering_angle
        self.control_velocity = control_msg.drive.speed'''
    

    def odom_callback(self,odom_msg):
        self.Vx = odom_msg.twist.twist.linear.x
        self.Vy = odom_msg.twist.twist.linear.y
        print("mmmmm")

    def imu_callback(self,imu_msg):
        self.yaw = imu_msg.orientation.z


    def waypoints_callback(self,waypoints_msg):
        print("a7a")
        #if len(waypoints_msg.markers[0].points)<2:
          #  return

        for point in waypoints_msg.markers[0].points:
            self.waypoints_x.append(point.x)
            self.waypoints_y.append(point.y)
        
        ### speed PID Controller ###
        self.target_sp=self.target_speed()
        self.vel_cmd = self.pid_control(self.target_sp,self.Vx)

    ### Stanley Steering Controller ###
       ## self.steer_cmd = self.pure_pursuit()

    ### publish control command ###
        self.control_command(self.steer_cmd,self.vel_cmd)

    def pid_control(self,target, current):
        """
        Proportional control for the speed.
        :param target: (float)
        :param current: (float)
        :return: (float)
        """
        self.d_error=(target-current)/self.dt
        self.i_error=(target-current)*self.dt
        self.p_error=target-current
        return self.Kp*self.p_error + self.Ki*self.i_error + self.Kd*self.d_error

    def target_speed(self):
        x1=self.waypoints_x[0]
        x2=self.waypoints_x[1]
        x3=self.waypoints_x[2]
        y1=self.waypoints_y[0]
        y2=self.waypoints_y[1]
        y3=self.waypoints_y[2]

        CO_Friction=0.6
        Gravity= 9.81

        c = (x1-x2)**2 + (y1-y2)**2
        a = (x2-x3)**2 + (y2-y3)**2
        b = (x3-x1)**2 + (y3-y1)**2
        ar = a**0.5
        br = b**0.5
        cr = c**0.5 
        r = ar*br*cr / ((ar+br+cr)*(-ar+br+cr)*(ar-br+cr)*(ar+br-cr))**0.5
 
        return math.sqrt(CO_Friction * Gravity * r)
    '''''
    def pure_control(self):
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
        '''

    def line_equation(self,x1,y1,x2,y2):

        dy = y2-y1
        dx = x2-x1 

        m = dy/dx
        b = y2 - m*x2

        return m ,b



    def control_command(self,steering,velocity):
        control_msg = AckermannDriveStamped()

        control_msg.drive.steering_angle = steering
        control_msg.drive.speed = velocity
    
        self.robot_control_pub.publish(control_msg)
       ## print("vvvvdxcv/dl/fn,jdsbf,m.ldsmf,sdcf")


    def pure_pursuit(self):
        
        #ld = math.sqrt( (self.x**2) + (self.y**2) ) ## lookahead distance
           
        #alpha = math.atan(self.y / self.x)
        
        if len(self.waypoints_x) <=1:
            self.x = self.waypoints_x[0]
            self.y = self.waypoints_y[0]

        elif len(self.waypoints_x) >1 and len(self.waypoints_x)<=2:
            self.x = self.waypoints_x[1]
            self.y = self.waypoints_y[1]
        
        else:
            self.x = self.waypoints_x[2]
            self.y = self.waypoints_y[2]



        

        self.alpha = math.atan(self.y / self.x) - self.yaw

        self.ld = math.sqrt( (self.x**2) + (self.y**2) ) ## lookahead distance
        if self.waypoints_y[1] >0:
            eta = math.atan( ( 2 * self.L *math.sin(self.alpha) ) / (self.ld) ) ## steering angle
        else:
            eta = -1 * math.atan( ( 2 * self.L *math.sin(self.alpha) ) / (self.ld) ) ## steering angle
        #r = self.L /(math.tan(eta))  ## radius
        


        #vector_look_ahead = [self.x, self.y]
        #vector_car = [np.cos(self.yaw), np.sin(self.yaw)]
        #corss_track_error = np.cross(vector_look_ahead, vector_car)
        #curvature = (2/self.ld/self.ld) * corss_track_error
        #eta = math.atan(curvature * self.L)
        


        '''
        k = 2*math.sin(alpha) / ld
        kvel = 0.3
        vel = math.sqrt((0.6 * 9.81) / k ) 

        ld = kvel * vel

        eta = math.atan( ( 2 * self.L *math.sin(alpha) ) / (ld)) ## steering angle 
        '''
  
        #kc =0.000000001
        ##  w2 = (2*0.5*sin(alpha))/l
        # w = 0.5*kc    ##change in heading
        
        #sigma = math.atan(l*kc)
        
        return eta

        
        



if __name__ == '__main__':
    try:
        control= pid_controller()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
