#!/usr/bin/python

import rospy
from movo_msgs.msg import *
from collections import deque
from relaxed_ik.msg import EEPoseGoals, JointAngles

'''
 This node adds latency between publishing and receiving 
 relaxedIK joint angle solutions by using another ROS topic
'''

import sys

r_vels = deque()
l_vels = deque()
sols = deque()
latency = 3

def r_vel_cb(data):
    r_vels.append(data)

def l_vel_cb(data):
    l_vels.append(data)

def relik_sol_cb(data):
    data.header.stamp = rospy.Time.now()
    sols.append(data)


if __name__ == "__main__":
    rospy.init_node('latency_node')

    relik_sol_sub = rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, relik_sol_cb)
    relik_sol_pub = rospy.Publisher('/delayed/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=10)



    if len(sys.argv) == 1:
        print("Latency: 3\n(Add latency argument to command line if needed)")
    else:
        latency = float(sys.argv[1])
        print("Latency: " + str(latency))

    while not rospy.is_shutdown():
        # if len(r_vels) != 0 and rospy.Time.now() - r_vels[0].header.stamp > rospy.Duration.from_sec(latency):
        #     r_vel_pub.publish(r_vels.popleft())
        # if len(l_vels) != 0 and rospy.Time.now() - l_vels[0].header.stamp > rospy.Duration.from_sec(latency):
        #     l_vel_pub.publish(l_vels.popleft())
        if len(sols) != 0 and rospy.Time.now() - sols[0].header.stamp > rospy.Duration.from_sec(latency):
            relik_sol_pub.publish(sols.popleft())
