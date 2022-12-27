#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rospy
import roslib
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO

#Define nombre de las entradas del puente H
ena = 18
in1 = 23
in2 = 24

enb = 19
in3 = 6
in4 = 5
#configura los pines segun el microprocesador Broadcom
GPIO.setmode(GPIO.BCM)
#configura los pines como salidas
GPIO.setup(ena,GPIO.OUT)
GPIO.setup(enb,GPIO.OUT)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
#Define las salidas PWM q
pwm_a = GPIO.PWM(ena,500)
pwm_b = GPIO.PWM(enb,500)
#inicializan los PWM con un duty Cicly de cero
pwm_a.start(0)
pwm_b.start(0)
# funciones de sentido de giro de los motores
def  Giro_Favor_Reloj_MotorA():
   GPIO.output(in1,False)
   GPIO.output(in2,True)

def Giro_Contra_Reloj_MotorA():
   GPIO.output(in1,True)
   GPIO.output(in2,False)

def  Giro_Favor_Reloj_MotorB():
   GPIO.output(in3,False)
   GPIO.output(in4,True)

def Giro_Contra_Reloj_MotorB():
   GPIO.output(in3,True)
   GPIO.output(in4,False)

heading = 360

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
        while not rospy.is_shutdown():
        
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()
                
    #############################################################
    def spinOnce(self):
    #############################################################
    
      # dx = (l + r) / 2
      # dr = (r - l) / w

      rospy.loginfo("dx, dr, w: [%f, %f, %f]"%(self.dx, self.dr, self.w))

      self.right = float(1.0) * float(self.dx) + self.dr * self.w / 2 
      self.left = float(1.0) * float(self.dx) - self.dr * self.w / 2

      rospy.loginfo("publishing: (%f, %f)", self.left, self.right) 
               
      self.pub_lmotor.publish(self.left)
      self.pub_rmotor.publish(self.right)
         
      self.ticks_since_target += 1

      if (self.right >= 0):
         velocidad_right = self.right * 100
         direccion_right = 'f'
      else:
         velocidad_right = -self.right * 100
         direccion_right = 'r'

      if (self.left >= 0):
         velocidad_left = self.left * 100
         direccion_left = 'f'
      else:
         velocidad_left = -self.left * 100
         direccion_left = 'r'

      if direccion_right == 'f':
         Giro_Favor_Reloj_MotorA()
         rospy.loginfo("motor A, CW, vel=%f", velocidad_right)
      elif direccion_right == 'r':
         Giro_Contra_Reloj_MotorA()
         rospy.loginfo("motor A, CCW, vel=%f", velocidad_right)

      pwm_a.ChangeDutyCycle(int(velocidad_right))

      if direccion_left == 'f':
         Giro_Favor_Reloj_MotorB()
         rospy.loginfo("motor B, CW, vel=%f", velocidad_left)
      elif direccion_left == 'r':
         Giro_Contra_Reloj_MotorB()
         rospy.loginfo("motor B, CW, vel=%f", velocidad_left)

      pwm_b.ChangeDutyCycle(int(velocidad_left))

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
    except rospy.ROSInterruptException:
        pass


#rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
