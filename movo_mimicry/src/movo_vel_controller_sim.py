#!/usr/bin/python

import rospy
import numpy as np
import math
import os
from movo_msgs.msg import *
from sensor_msgs.msg import JointState
from RelaxedIK.relaxedIK import get_relaxedIK_from_info_file
from relaxed_ik.msg import EEPoseGoals, JointAngles
from std_msgs.msg import Int8, Int16
from geometry_msgs.msg import Twist, Point, Pose

##
# Use this node for Movo velocity control in simulation
# The only difference is that collision detection is turned off
##

## Changes from normal:
# - Commented out collision checking
# - r_arm_js and l_arm_js set to [0,0,..,0] instead of None

r_vel_pub = None
r_arm_js = None
l_arm_js = None
relik_update_time = 0
rik = None
i = 0


# TODO: Update clutch / grab

def check_and_move_arms(right_sol, left_sol):
    '''
    Calls functions to move right and left arms based on solutions
    '''
    global r_arm_js, l_arm_js, relik_update_time
    s = 3.0 #4.0 works well
    right_vel = np.array(right_sol) - np.array(r_arm_js)
    right_vel = right_vel*s
    left_vel = np.array(left_sol) - np.array(l_arm_js)
    left_vel = left_vel*s
    if (rospy.get_time() - relik_update_time <= 0.01):
        return
    # if collision_detected(right_vel, left_vel) is True:
    #     relik_update_time = rospy.get_time()
    #     colliding = Int8()
    #     colliding.data = 1
    #     collsion_pub.publish(colliding)
    #     return
    colliding = Int8()
    colliding.data = 0
    collsion_pub.publish(colliding)
    # move_right_arm(right_sol)
    # move_left_arm(left_sol)
    move_arm('r', right_sol)
    move_arm('l', left_sol)
    relik_update_time = rospy.get_time()


def move_arm(arm, sol):
    '''
    Move right or left arm using joint states and angular velocity commands
    :param arm: 'r' or 'l' for right or left arm
    :param sol: desired arm state
    '''
    global r_vel_pub, l_vel_pub, r_arm_js, l_arm_js
    vel_pub = r_vel_pub
    arm_js = r_arm_js
    s = 3.0

    if arm == 'r':
        if r_vel_pub is None:
            r_vel_pub = rospy.Publisher('/movo/right_arm/angular_vel_cmd/', JacoAngularVelocityCmd7DOF, queue_size=10)
        vel_pub = r_vel_pub
        arm_js = r_arm_js
    elif arm == 'l':
        if l_vel_pub is None:
            l_vel_pub = rospy.Publisher('/movo/left_arm/angular_vel_cmd/', JacoAngularVelocityCmd7DOF, queue_size=10)
        vel_pub = l_vel_pub
        arm_js = l_arm_js
    else:
        print("No arm chosen to move")
        return

    if arm_js is None:
        print("No joints")
        return

    vel = np.array(sol) - np.array(arm_js)

    # print(vel)
    vel = vel*s
    for i in range(7):
        if i <= 3 and vel[i] > 0.69:
            vel[i] = 0.69
        elif i > 3 and vel[i] > 0.92:
            vel[i] = 0.92
        elif i <= 3 and vel[i] < -0.69:
            vel[i] = -0.69
        elif i > 3 and vel[i] < -0.92:
            vel[i] = -0.92
        # print("{} {}".format(i, vel[i]))
    # print(vel)


    vel = (vel/math.pi)*180
    # print(vel)

    cmd = JacoAngularVelocityCmd7DOF()
    cmd.header.stamp = rospy.Time.now()
    cmd.theta_shoulder_pan_joint = vel[0]
    cmd.theta_shoulder_lift_joint = vel[1]
    cmd.theta_arm_half_joint = vel[2]
    cmd.theta_elbow_joint = vel[3]
    cmd.theta_wrist_spherical_1_joint = vel[4]
    cmd.theta_wrist_spherical_2_joint = vel[5]
    cmd.theta_wrist_3_joint = vel[6]
    # print(cmd)
    vel_pub.publish(cmd)


def move_right_arm(sol):
    '''
    Moves right arm according to joint states. Currently unused.
    '''
    global r_vel_pub, r_arm_js
    s = 3.0

    if r_vel_pub is None:
        r_vel_pub = rospy.Publisher('/movo/right_arm/angular_vel_cmd/', JacoAngularVelocityCmd7DOF, queue_size=10)

    if r_arm_js is None:
        print("No joints")
        return
    vel = np.array(sol) - np.array(r_arm_js)

    vel = vel*s
    vel = (vel/math.pi)*180

    cmd = JacoAngularVelocityCmd7DOF()
    cmd.header.stamp = rospy.Time.now()
    cmd.theta_shoulder_pan_joint = vel[0]
    cmd.theta_shoulder_lift_joint = vel[1]
    cmd.theta_arm_half_joint = vel[2]
    cmd.theta_elbow_joint = vel[3]
    cmd.theta_wrist_spherical_1_joint = vel[4]
    cmd.theta_wrist_spherical_2_joint = vel[5]
    cmd.theta_wrist_3_joint = vel[6]
    # print(cmd)
    r_vel_pub.publish(cmd)


def move_left_arm(sol):
    '''
    Moves left arm according to joint states. Currently unused.
    '''
    global l_vel_pub, l_arm_js
    s = 3.0

    if l_vel_pub is None:
        l_vel_pub = rospy.Publisher('/movo/left_arm/angular_vel_cmd/', JacoAngularVelocityCmd7DOF, queue_size=10)

    if l_arm_js is None:
        print("No joints")
        return
    vel = np.array(sol) - np.array(l_arm_js)
    #  print(vel)
    vel = vel*s
    vel = (vel/math.pi)*180

    cmd = JacoAngularVelocityCmd7DOF()
    cmd.header.stamp = rospy.Time.now()
    cmd.theta_shoulder_pan_joint = vel[0]
    cmd.theta_shoulder_lift_joint = vel[1]
    cmd.theta_arm_half_joint = vel[2]
    cmd.theta_elbow_joint = vel[3]
    cmd.theta_wrist_spherical_1_joint = vel[4]
    cmd.theta_wrist_spherical_2_joint = vel[5]
    cmd.theta_wrist_3_joint = vel[6]
    # print(cmd)
    l_vel_pub.publish(cmd)


def collision_detected(right_vel, left_vel):
    '''
    Checks if a collision occurs when right and left
    joint angles are moved by right_vel and left_vel
    '''
    global r_arm_js, l_arm_js, rik, relik_update_time

    right_joints = np.array(r_arm_js)
    right_joints = right_joints + right_vel
    joints = [0.0] # Linear joint, put 0 as it is not moved
    joints = np.concatenate((joints, right_joints))

    left_joints = np.array(l_arm_js)
    left_joints = left_joints + left_vel
    joints = np.concatenate((joints, left_joints))

    # joints = np.concatenate((joints, [0.0, 0.0]))

    col_val = rik.vars.collision_graph.get_collision_score_of_state(joints)
    b = rik.vars.collision_graph.b_value
    b = 5
    rik.vars.collision_graph.c.draw_all()

    if col_val >= b:
        print("%f, %f: Collision" % (col_val, b))
        return True
    else:
        print("%f, %f: No Collision" % (col_val, b))
        return False


def right_js_callback(data):
    global r_arm_js
    r_arm_js = data.position
    return


def left_js_callback(data):
    global l_arm_js
    l_arm_js = data.position
    return


def relik_sol_cb(data):
    '''
    Gets data from '/relaxed_ik/joint_angle_solutions' topic
    and sends right arm points to velocity controller
    '''
    global relik_update_time, r_arm_js
    global rik, i

    right_sol = data.angles.data[1:8]
    left_sol = data.angles.data[8:15]
    check_and_move_arms(right_sol, left_sol)


def grab_r_cb(data):
    cmd = GripperCmd()
    cmd.position = 1 - data.data  # 0 - 0.9
    cmd.speed = 0.5
    gripper_pub_r.publish(cmd)


def grab_l_cb(data):
    cmd = GripperCmd()
    cmd.position = 1 - data.data  # 0 - 0.9
    cmd.speed = 0.5
    gripper_pub_l.publish(cmd)


def base_cb(data):
    '''
    Moves the base according to callback data using '/movo/teleop/cmd_vel/' topic
    '''
    base_cmd = Twist()
    val_x = 0.07
    val_y = 0.07
    val_z = 0.13
    num = data.data
    if num % 10 == 1:
        base_cmd.angular.z = -1*val_z
    elif num % 10 == 2:
        base_cmd.angular.z = 1*val_z
    else:
        base_cmd.angular.z = 0
    num /= 10
    if num % 10 == 1:
        base_cmd.linear.y = -1*val_y
    elif num % 10 == 2:
        base_cmd.linear.y = 1*val_y
    else:
        base_cmd.linear.y = 0

    if num / 10 == 2:
        base_cmd.linear.x = 1*val_x
    elif num / 10 == 3:
        base_cmd.linear.x = -1*val_x
    else:
        base_cmd.linear.x = 0
    # print(base_cmd)

    base_pub.publish(base_cmd)

if __name__ == "__main__":
    rospy.init_node('movo_vel_controller')
    r_vel_pub = rospy.Publisher('/movo/right_arm/angular_vel_cmd/', JacoAngularVelocityCmd7DOF, queue_size=10)
    r_js_sub = rospy.Subscriber('/movo/right_arm/joint_states', JointState, right_js_callback)
    l_vel_pub = rospy.Publisher('/movo/left_arm/angular_vel_cmd/', JacoAngularVelocityCmd7DOF, queue_size=10)
    l_js_sub = rospy.Subscriber('/movo/left_arm/joint_states', JointState, left_js_callback)
    relik_sol_sub = rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, relik_sol_cb)
    grab_sub_r = rospy.Subscriber('/vive_controller/grab_r', Int8, grab_r_cb)
    grab_sub_l = rospy.Subscriber('/vive_controller/grab_l', Int8, grab_l_cb)
    gripper_pub_r = rospy.Publisher('/movo/right_gripper/cmd', GripperCmd, queue_size=10)
    gripper_pub_l = rospy.Publisher('/movo/left_gripper/cmd', GripperCmd, queue_size=10)
    collsion_pub = rospy.Publisher('/vel_controller/is_colliding', Int8, queue_size=10)
    vive_base_sub = rospy.Subscriber('/vive_controller/base', Int16, base_cb)
    base_pub = rospy.Publisher('/movo/teleop/cmd_vel/', Twist, queue_size=10)


    # r_arm_js = [0]*7
    # l_arm_js = [0]*7
    path_to_src = os.path.join(os.path.dirname(__file__), '../../relaxed_ik/src')
    print(path_to_src)
    rik = get_relaxedIK_from_info_file(path_to_src, preconfig=True)
    relik_update_time = rospy.get_time()
    print("spinning...")
    rospy.spin()
