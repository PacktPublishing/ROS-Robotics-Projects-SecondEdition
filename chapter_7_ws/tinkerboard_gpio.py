#!/usr/bin/env python
 
 import ASUS.GPIO as GPIO
 import time
 import rospy
 
 GPIO.setwarnings(False)
 GPIO.setmode(GPIO.ASUS)
 
 LED  = 49
 
 GPIO.setup(LED,GPIO.OUT)
 
 def ledBlink(data):
  if data.data = true:
     GPIO.output(LED, GPIO.HIGH)
     time.sleep(0.5)
     GPIO.output(LED, GPIO.LOW)
     time.sleep(0.5)
  else:
     GPIO.cleanup()
 
 if __name__ == '__main__':
     rospy.init_node('Led_blink', anonymous=True)
     rospy.Subscriber("/led_status", Bool, ledBlink)
     rospy.spin()
