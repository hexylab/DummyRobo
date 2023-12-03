#!/usr/bin/env python

from kinematics_lib import *
import time
import rospy
import roslib
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

myDummy = DummyArm(initial_q1=0, initial_q2=90, initial_q3=-90, initial_ee=0)

geo_list = [ [137,0,141,1],
             [20,190,50,1],
             [20,190,50,0],
             [20,190,130,0],
             [20,-190,130,0],
             [20,-190,50,0],
             [20,-190,50,1] ]

def calcJointState(x,y,z,e):
    js = JointState()
    js.header = Header()
    js.name = ['joint_1','joint_2','joint_3','joint_4','endeffector']
    #js.header.stamp = rospy.Time.now()
    
    a1,a2,a3 = myDummy.inverseKinematics(x,y,z)
    myDummy.updateJointAngles(a1,a2,a3)
    myDummy.updateEndEffector(e)
    q1,q2,q3,q4,ee = myDummy.getRadJointState()

    js.position = [q1,q2,q3,q4,ee]
    js.velocity = []
    js.effort = []

    return js

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('dummy_joint_publisher')
    rate = rospy.Rate(10)
    geo_count = 0
    while not rospy.is_shutdown():
        x,y,z,e = geo_list[geo_count]
        js = calcJointState(x,y,z,e)
        js.header.stamp = rospy.Time.now()
        rospy.loginfo(js)
        pub.publish(js)
        time.sleep(2)
        geo_count = geo_count + 1
        if(geo_count == 7):
            geo_count = 0
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
