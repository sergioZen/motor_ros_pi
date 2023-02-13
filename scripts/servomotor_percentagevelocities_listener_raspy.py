#!/usr/bin/env python3

import math
from math import sin, cos, pi
import pigpio
import time
from time import sleep
from adafruit_servokit import ServoKit

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf2_ros as tf2

from diffbot_msgs.msg import WheelsCmdStamped

servoPINLeft = 18
servoPINRight = 17

###### main loop  ######
# ServoMotor 3606HB: 0.16 s (4.8V) ¦ 0.14 s (6.0V)
# Using hardware:            PWM
# Idle timeout:         Disabled
# Number of servos:            1
# Servo cycle time:        20000us
# Pulse width units:          10us
# Minimum width value:        0.8 (800us)
# Maximum width value:        2.2 (2200us)
# Output levels:          Normal
# Real Voltage received in the servomotor: 5 V
#########################################################

def mapFromTo(x,a,b,c,d):
   y=(x-a)/(b-a)*(d-c)+c
   return y   

#############################################################
#############################################################
class TwistToMotors():
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("twist_to_motors")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
    
        self.w = rospy.get_param("~base_width", 0.2)

        rospy.Subscriber("diffbot/motor_left", Int32, self.wheelCmdLeftVelocityCallback)
        rospy.Subscriber("diffbot/motor_right", Int32, self.wheelCmdRightVelocityCallback)

        # Set channels to the number of servo channels on your kit.
        # 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
        self.kit = ServoKit(channels=16)

        throttle_range = {-1.0, 1.0}

        self.kit.continuous_servo[0].min_pulse = 500
        self.kit.continuous_servo[0].max_pulse = 2500
        self.kit.continuous_servo[1].min_pulse = 500
        self.kit.continuous_servo[1].max_pulse = 2500

        """
        self.kit.continuous_servo[0].throttle = 1
        self.kit.continuous_servo[1].throttle = -1
        time.sleep(1)
        self.kit.continuous_servo[0].throttle = -1
        self.kit.continuous_servo[1].throttle = 1
        time.sleep(1)
        self.kit.continuous_servo[0].throttle = 0
        self.kit.continuous_servo[1].throttle = 0
        time.sleep(1)        
        """
    
        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = 0
        self.right = 0

        self.duty_left = 0
        self.duty_right = 0

        # self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        # self.odom_broadcaster = tf2.TransformBroadcaster()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.1
        self.vy = -0.1
        self.vth = 0.1

    def sendOdometry(self):
      current_time = rospy.Time.now()

      self.vx = self.dx
      self.vy = self.dr
      self.th = self.w

      # compute odometry in a typical way given the velocities of the robot
      dt = (current_time - last_time).to_sec()
      delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
      delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
      delta_th = self.vth * dt

      x += delta_x
      y += delta_y
      th += delta_th

      # since all odometry is 6DOF we'll need a quaternion created from yaw
      odom_quat = tf2.transformations.quaternion_from_euler(0, 0, th)

      # first, we'll publish the transform over tf
      self.odom_broadcaster.sendTransform(
         (x, y, 0.),
         odom_quat,
         current_time,
         "base_link",
         "odom"
      )

      # next, we'll publish the odometry message over ROS
      odom = Odometry()
      odom.header.stamp = current_time
      odom.header.frame_id = "odom"

      # set the position
      odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

      # set the velocity
      odom.child_frame_id = "base_link"
      odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

      # publish the message
      self.odom_pub.publish(odom)

      last_time = current_time 

    #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks

        self.pi = pigpio.pi()
        self.pi.set_servo_pulsewidth(servoPINLeft, 0)
        self.pi.set_servo_pulsewidth(servoPINRight, 0)
    
        ###### main loop  ######
        while not rospy.is_shutdown():
        
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                # self.spinOnceTwist()
                self.spinOnceWheelCmdStamped()
                r.sleep()
            idle.sleep()

    #############################################################
    def spinOnceWheelCmdStamped(self):
    #############################################################

      rospy.loginfo("left, right: [%f, %f]"%(self.duty_left, self.duty_right))
    
      self.ticks_since_target += 1

      if (self.duty_left != 0):
        self.duty_left = self.duty_left

      self.kit.continuous_servo[0].throttle = self.duty_left / 20
      self.kit.continuous_servo[1].throttle = self.duty_right / 20
 
    def wheelCmdLeftVelocityCallback(self, cmd_msg):

        wheel_cmd_velocity_left_ = cmd_msg.data

        # From 500 to 1400 CW:
        # From 1600 to 2500 CCW:
        # For stoppping: 1500 (or 0)
        if (wheel_cmd_velocity_left_ > 100):
            wheel_cmd_velocity_left_ = 100

        if (wheel_cmd_velocity_left_ < -100):
            wheel_cmd_velocity_left_ = -100   

        if (wheel_cmd_velocity_left_ != 0):
          wheel_cmd_velocity_left_ = wheel_cmd_velocity_left_ / 100

        self.duty_left = wheel_cmd_velocity_left_ 

        self.ticks_since_target = 0            

    def wheelCmdRightVelocityCallback(self, cmd_msg):

        wheel_cmd_velocity_right_ = cmd_msg.data        

        # From 500 to 1400 CW:
        # From 1600 to 2500 CCW:
        # For stoppping: 1500 (or 0)
        # (in case of right motor is located in mirror position)
        if (wheel_cmd_velocity_right_ > 100):
            wheel_cmd_velocity_right_ = 100

        if (wheel_cmd_velocity_right_ < -100):
            wheel_cmd_velocity_right_ = -100

        if (wheel_cmd_velocity_right_ != 0):
          wheel_cmd_velocity_right_ = wheel_cmd_velocity_right_ / 100                       

        self.duty_right = -wheel_cmd_velocity_right_

        self.ticks_since_target = 0            

#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    try:
        twistToMotors = TwistToMotors()
        twistToMotors.spin()
    except rospy.ROSInterruptException:
        pass

#rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
