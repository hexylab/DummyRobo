#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import roslib
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from dummy_ros.msg import PointEeState

def main():
    rospy.init_node("test_node")
    pub = rospy.Publisher("pointee_states", PointEeState, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pe = PointEeState()
        pe.position = Point()
        pe.position.x = float(input("X([mm]):"))
        pe.position.y = float(input("Y([mm]):"))
        pe.position.z = float(input("Z([mm]):"))
        pe.ee = int(input("Open(1) or Close(0):"))
        pub.publish(pe)
        rospy.loginfo("Message '{}' published".format(pe))
        rate.sleep()

if __name__ == "__main__":
    main()
