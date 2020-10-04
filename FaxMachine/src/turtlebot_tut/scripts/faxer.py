#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def faxer():
    rospy.init_node('faxer', anonymous=True)
    pub = rospy.Publisher('fax_line', String, queue_size=10)
    rate = rospy.Rate(10) # 10 Hertz
    
    while not rospy.is_shutdown():
        message = "hello world"
        pub.publish(message)
        rate.sleep()
    
if name == "__main__":
    try:
        faxer()
    except rospy.ROSInterruptException
        pass