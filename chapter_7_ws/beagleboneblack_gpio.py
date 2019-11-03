#!/usr/bin/env python
 
 import Adafruit_BBIO.GPIO as GPIO
 import time
 import rospy
 
 LED  = "P8_10"
 
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
