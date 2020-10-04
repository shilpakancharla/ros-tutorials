#!/usr/bin/env python
import rospy
from std_msgs import String

def fax_handler(data):
    rospy.loginfo(data.data)

def printer():
    rospy.init_node('printer', anonymous=True)
    rospy.Subscriber('fax_line', String, fax_handler)
    
    rospy.spin()
    
if name == "__main__":
    printer()