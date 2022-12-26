#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
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

def callback(data):
   cmd = data.data.lower()
   motor = cmd[0]
   direccion =cmd[1]
   velocidad =cmd[2:5]

   if motor == "a":
         if direccion == "f":
                  Giro_Favor_Reloj_MotorA()
                  print("motor A, CW, vel="+velocidad)
         elif direccion== "r":
                  Giro_Contra_Reloj_MotorA()
                  print("motor A, CCW, vel="+velocidad)
         else:
                  print("comando no reconocido")
         pwm_a.ChangeDutyCycle(int(velocidad))
         print

   elif motor == "b":
         if direccion == "f":
                  Giro_Favor_Reloj_MotorB()
                  print("motor B, CW, vel="+velocidad)
         elif direccion == "r":
                  Giro_Contra_Reloj_MotorB()
         else:
                  print("comando no reconocido")
         pwm_b.ChangeDutyCycle(int(velocidad))
         print
   else:
         print
         print("comando no reconocido")
         print
      
   rospy.loginfo( 'heading %d', heading )
   
   #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():

   rospy.init_node('listener', anonymous=True)

   rospy.Subscriber('direction', String, callback)

   rospy.spin()

if __name__ == '__main__':
    listener()
