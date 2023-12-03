#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Title: motion_planner_node
# Description: Simple motion planner which calculates a JointState from a Point of end-effector

from kinematics_lib import *
import time
import rospy
import roslib
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from dummy_ros.msg import PointEeState

class ikNode():
    def __init__(self):
        self.sub = rospy.Subscriber('pointee_states', PointEeState, self.callback)
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.myDummy = DummyArm(initial_q1=0, initial_q2=90, initial_q3=-90, initial_ee=0)

    def callback(self, data):
        self.publish(data)

    def publish(self, data):
        x,y,z,e = self.fetchPointEeState(data)
        js = self.calcJointState(x,y,z,e)
        js.header.stamp = rospy.Time.now()
        rospy.loginfo(js)
        self.pub.publish(js)

    def fetchPointEeState(self, data):
        x = data.position.x
        y = data.position.y
        z = data.position.z
        e = data.ee
        return x,y,z,e    

    def calcJointState(self, x,y,z,e):
        js = JointState()
        js.header = Header()
        js.name = ['joint_1','joint_2','joint_3','joint_4','endeffector']

        a1,a2,a3 = self.myDummy.inverseKinematics(x,y,z)
        self.myDummy.updateJointAngles(a1,a2,a3)
        self.myDummy.updateEndEffector(e)
        q1,q2,q3,q4,ee = self.myDummy.getRadJointState()

        js.position = [q1,q2,q3,q4,ee]
        js.velocity = []
        js.effort = []

        return js

if __name__ == '__main__':
    rospy.init_node('invert_kinematics_node')
    time.sleep(3.0)
    node = ikNode()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
