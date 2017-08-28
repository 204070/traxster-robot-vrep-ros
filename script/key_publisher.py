#!/usr/bin/env python

import sys, select, tty, termios
import rospy
from std_msgs.msg import String

def oyakeypublish():
    rospy.init_node("keyboard_driver", anonymous=True)
    key_pub = rospy.Publisher("keys", String, queue_size=1)
    rate = rospy.Rate(100)
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print("Pulblishing keystrokes. Press Ctrl-C to exit...")
    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key_pub.publish(sys.stdin.read(1))
        rate.sleep()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)


if __name__ == "__main__":
    oyakeypublish()
   