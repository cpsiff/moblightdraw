#!/usr/bin/env python3

import rospy
import numpy as np
from mobrob_util.msg import ME439Color
import qwiic_led_stick
import sys
import traceback
   

def talker():
    sub_color = rospy.Subscriber('/led_color', ME439Color, set_led_color)  

    if my_stick.begin() == False:
        print("\nThe Qwiic LED Stick isn't connected to the sytsem. Please check your connection", \
            file=sys.stderr)
        return

    print("\nLED Stick ready!")  

    my_stick.set_all_LED_brightness(15)

    rospy.spin()
    

def set_led_color(msg_in):
    global my_stick

    my_stick.set_all_LED_color(int(msg_in.red), int(msg_in.green), int(msg_in.blue))


my_stick = qwiic_led_stick.QwiicLEDStick()

if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass
