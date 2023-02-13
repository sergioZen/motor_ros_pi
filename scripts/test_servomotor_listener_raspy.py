#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

import pigpio
import time

servoPINLeft = 18
servoPINRight = 17

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
    
        self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Float32, queue_size=10)
        self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32, queue_size=10)

        #rospy.Subscriber('twist', Twist, self.twistCallback)
        rospy.Subscriber("cmd_vel", Twist, self.twistCallback)
    
        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = 0
        self.right = 0
         
        
    #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
    
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

        freq = 1 / 50

        # Stop at:  
        stop_duty = (2.2 - 0.8)/2 + 0.8 # 1.5 ms for center

        pi = pigpio.pi()
        pi.set_servo_pulsewidth(servoPINLeft, 0)
        pi.set_servo_pulsewidth(servoPINRight, 0)
        time.sleep(2)

        for i in range(5,15):
            duty = i*100
            pi.set_servo_pulsewidth(servoPINLeft, duty)
            pi.set_servo_pulsewidth(servoPINRight, duty)
            time.sleep(2)
            
        pi.set_servo_pulsewidth(servoPINLeft, 0)
        pi.set_servo_pulsewidth(servoPINRight, 0)
        time.sleep(2)

        pi.set_servo_pulsewidth(servoPINLeft, 1500)
        pi.set_servo_pulsewidth(servoPINRight, 1500)
        time.sleep(2)  

        pi.set_servo_pulsewidth(servoPINLeft, 0)
        pi.set_servo_pulsewidth(servoPINRight, 0)

        time.sleep(2)              

        for i in range(16,25):
            duty = i*100
            pi.set_servo_pulsewidth(servoPINLeft, duty)
            pi.set_servo_pulsewidth(servoPINRight, duty)
            time.sleep(2)
            
        pi.set_servo_pulsewidth(servoPINLeft, 0)
        pi.set_servo_pulsewidth(servoPINRight, 0)

        time.sleep(2)

    #############################################################
    def twistCallback(self,msg):
    #############################################################
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        rospy.loginfo("Received a /cmd_vel message!")
        rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y

#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    try:
        twistToMotors = TwistToMotors()
        twistToMotors.spin()
        twistToMotors.p.stop()
    except rospy.ROSInterruptException:
        pass


#rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
