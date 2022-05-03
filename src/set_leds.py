#!/usr/bin/env python3

import rospy
import numpy as np
from mobrob_util.msg import ME439Color
import qwiic_led_stick
import sys
import traceback
   

def talker():
    rospy.init_node('set_leds', anonymous=False)

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

    red_list = [msg_in.red]*10
    green_list = [msg_in.green]*10
    blue_list = [msg_in.blue]*10

    for i in range(10):
        if i >= msg_in.width:
            red_list[i] = 0
            green_list[i] = 0
            blue_list[i] = 0

    my_stick.set_all_LED_unique_color(red_list, green_list, blue_list, 10)


my_stick = qwiic_led_stick.QwiicLEDStick()

if __name__ == '__main__':
    try: 
        talker()
        my_stick.LED_off()
    except rospy.ROSInterruptException:
        my_stick.LED_off()
        pass