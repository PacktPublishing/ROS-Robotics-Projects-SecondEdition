#!/usr/bin/env python
 
 from gpiozero import LED
 import time
 import rospy
 
 LED  = LED(17)
 
 GPIO.setup(LED,GPIO.OUT)
 
 def ledBlink(data):
  if data.data = true:
     LED.blink()
  else:
     print("Waiting for blink command...")
 
 if __name__ == '__main__':
     rospy.init_node('Led_blink', anonymous=True)
     rospy.Subscriber("/led_status", Bool, ledBlink)
     rospy.spin()
