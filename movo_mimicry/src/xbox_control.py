#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose, Transform
from movo_msgs.msg import GripperCmd
from std_msgs.msg import Bool
from relaxed_ik.msg import EEPoseGoals
import RelaxedIK.Utils.transformations as T
from inputs import get_gamepad
import sys
import multiprocessing
import time
import os, signal
from movo_mimicry.msg import XboxInputs
import numpy as np
import transforms3d as t3d

# ---
# Takes Xbox controller inputs from ROS topics
# Publishes bimanual movement information for the Movo
# ---


rospy.init_node('xbox_driver')

position_r = [0.,0.,0.]
rotation_r = [1.,0.,0.,0.]

position_l = [0.,0.,0.]
rotation_l = [1.,0.,0.,0.]

pos_step = 0.001
rot_step = 0.005
pos_add_r = [0.,0.,0.]

inputs = [0] * 19
stick_limit = 17000.0

prev_grab_input = 0
current_grab_val = 0

prev_grab_input_r = 0
current_grab_val_r = 0

prev_mode = 0
control_mode = 0

control_side = 0 ## 0 - left | 1 - right

transform = [[  1.,  0.,   0.,  .5],
       [  0.,   1.,   0.,  .5],
       [  0.,   0.,   1.,  .5],
       [  0.,   0.,   0.,   1.]]
transform_rot = list(T.euler_from_quaternion(rotation_l))


def xbox_cb(data):
    global inputs
    inputs = data.inputs

def grab_l(val):
    global prev_grab_input, current_grab_val
    if val == 1 and val != prev_grab_input:
        if current_grab_val == 0:
            current_grab_val = 0.9
        else:
            current_grab_val = 0
    prev_grab_input = val

    cmd = GripperCmd()
    cmd.position = current_grab_val  # 0 - 0.9
    cmd.speed = 0.5
    gripper_pub_l.publish(cmd)

def grab_r(val):
    global prev_grab_input_r, current_grab_val_r
    if val == 1 and val != prev_grab_input_r:
        if current_grab_val_r == 0:
            current_grab_val_r = 0.9
        else:
            current_grab_val_r = 0
    prev_grab_input_r = val

    cmd = GripperCmd()
    cmd.position = current_grab_val_r  # 0 - 0.9
    cmd.speed = 0.5
    gripper_pub_r.publish(cmd)

ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=5)
xbox_outputs_sub = rospy.Subscriber('/xbox_inputs', XboxInputs, xbox_cb)
gripper_pub_l = rospy.Publisher('/movo/left_gripper/cmd', GripperCmd, queue_size=10)
gripper_pub_r = rospy.Publisher('/movo/right_gripper/cmd', GripperCmd, queue_size=10)
transform_pub = rospy.Publisher('/control_plane_tf', Transform, queue_size=10)

def limit_val(val):
    if (abs(val) > 0.00016):
        return val
    else:
        return 0

def limit_val_r(val):
    if (abs(val) > 0.0008):
        return val
    else:
        return 0

control_mode_strings = ["Normal", "One Arm", "Point with Sticks", "Mimicked Control", "One Arm with Rotation", "Constrained Plane"]

r = rospy.Rate(50)
print(control_mode_strings[control_mode])
while not rospy.is_shutdown():


    grab_l(inputs[2])
    grab_r(inputs[5])

    if inputs[18] == 1 and inputs[18] != prev_mode:
        prev_mode = inputs[18]
        control_mode += 1
        control_mode %= 6
        print(control_mode_strings[control_mode])
    prev_mode = inputs[18]

    ## Normal
    if control_mode == 0:
        # ## left stick - left arm y
        # if inputs[0] > stick_limit:
        #     position_l[1] -= pos_step
        # elif inputs[0] < -1*stick_limit:
        #     position_l[1] += pos_step*()

        ## left stick - left arm y
        position_l[1] -= limit_val(pos_step*(inputs[0]/stick_limit))

        ## left stick - left arm x
        # if inputs[1] > stick_limit:
        #     position_l[0] -= pos_step
        # elif inputs[1] < -1 * stick_limit:
        #     position_l[0] += pos_step

        ## left stick - left arm x
        position_l[0] -= limit_val(pos_step*(inputs[1]/stick_limit))


        ## left button - left arm up
        if inputs[12] == 1:
            position_l[2] += pos_step

        ## left trigger - left arm down
        if inputs[13] > 700:
            position_l[2] -= pos_step

        ## Dpad left/right- left arm rotation left/right
        if inputs[6] > 0:
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[2] -= rot_step
            rotation_l = T.quaternion_from_euler(euler[0], euler[1], euler[2])
        elif inputs[6] < 0:
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[2] += rot_step
            rotation_l = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        ## Dpad up/down - left arm rotation up/down
        if inputs[7] > 0:
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[1] += rot_step
            rotation_l = T.quaternion_from_euler(euler[0], euler[1], euler[2])
        elif inputs[7] < 0:
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[1] -= rot_step
            rotation_l = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        if inputs[16] > 0:
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[0] += rot_step
            rotation_l = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        ## right stick - right arm y
        # if inputs[3] > stick_limit:
        #     position_r[1] -= pos_step
        # elif inputs[3] < -1*stick_limit:
        #     position_r[1] += pos_step

        position_r[1] -= limit_val(pos_step*(inputs[3]/stick_limit))


        ## right stick - right arm x
        # if inputs[4] > stick_limit:
        #     position_r[0] -= pos_step
        # elif inputs[4] < -1*stick_limit:
        #     position_r[0] += pos_step

        position_r[0] -= limit_val(pos_step*(inputs[4]/stick_limit))



        ## right button - right arm up
        if inputs[14] == 1:
            position_r[2] += pos_step

        ## right trigger - right arm down
        if inputs[15] > 700:
            position_r[2] -= pos_step

        ## X/B Button - right arm rotation
        if inputs[8] > 0:
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[2] += rot_step
            rotation_r = T.quaternion_from_euler(euler[0], euler[1], euler[2])
        if inputs[10] > 0:
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[2] -= rot_step
            rotation_r = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        ## Y/A - right arm rotation
        if inputs[11] > 0:
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[1] -= rot_step
            rotation_r = T.quaternion_from_euler(euler[0], euler[1], euler[2])
        if inputs[9] > 0:
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[1] += rot_step
            rotation_r = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        if inputs[17] > 0:
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[0] -= rot_step
            rotation_r = T.quaternion_from_euler(euler[0], euler[1], euler[2])

    ## Analog sticks XY and YZ plane
    elif control_mode == 1:
        if inputs[12] == 1:
            control_side = 0
            print("Controlling left arm")
        if inputs[14] == 1:
            control_side = 1
            print("Controlling right arm")
        if control_side == 0:
            position = position_l
            rotation = rotation_l
        else:
            position = position_r
            rotation = rotation_r

        ## left stick - arm y - left/right
        position[1] -= limit_val(pos_step * (inputs[0] / stick_limit))

        ## left stick - arm x - forward/back
        position[0] -= limit_val(pos_step * (inputs[1] / stick_limit))

        ## right stick - arm z - forward/back
        position[2] -= limit_val(pos_step*(inputs[4]/stick_limit))

        ## Dpad left/right- arm rotation left/right
        if inputs[6] > 0:
            euler = list(T.euler_from_quaternion(rotation))
            euler[2] -= rot_step
            rotation = T.quaternion_from_euler(euler[0], euler[1], euler[2])
        elif inputs[6] < 0:
            euler = list(T.euler_from_quaternion(rotation))
            euler[2] += rot_step
            rotation = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        ## Dpad up/down - arm rotation up/down
        if inputs[7] > 0:
            euler = list(T.euler_from_quaternion(rotation))
            euler[1] += rot_step
            rotation = T.quaternion_from_euler(euler[0], euler[1], euler[2])
        elif inputs[7] < 0:
            euler = list(T.euler_from_quaternion(rotation))
            euler[1] -= rot_step
            rotation = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        if inputs[16] > 0:
            euler = list(T.euler_from_quaternion(rotation))
            euler[0] += rot_step
            rotation = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        if control_side == 0:
            position_l = position
            rotation_l = rotation
        else:
            position_r = position
            rotation_r = rotation

    ## Pointing with sticks
    elif control_mode == 2:
        ## Pointing with sticks

        ## left stick - left/right rotation
        euler = list(T.euler_from_quaternion(rotation_l))
        euler[2] += limit_val_r(rot_step * (inputs[0] / stick_limit))
        rotation_l = T.quaternion_from_euler(euler[0], euler[1], euler[2])
        ## left stick - up/down rotation
        euler = list(T.euler_from_quaternion(rotation_l))
        euler[1] -= limit_val_r(rot_step * (inputs[1] / stick_limit))
        rotation_l = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        ## left button - left arm up
        if inputs[12] == 1:
            position_l[2] += pos_step

        ## left trigger - left arm down
        if inputs[13] > 700:
            position_l[2] -= pos_step

        ## Dpad left/right- left arm rotation left/right
        if inputs[6] > 0:
            position_l[1] -= pos_step
        elif inputs[6] < 0:
            position_l[1] += pos_step
        ## Dpad up/down - left arm rotation up/down
        if inputs[7] > 0:
            position_l[0] -= pos_step
        elif inputs[7] < 0:
            position_l[0] += pos_step

        if inputs[16] > 0:
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[0] += rot_step
            rotation_l = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        ## right stick - right arm y
        euler = list(T.euler_from_quaternion(rotation_r))
        euler[2] += limit_val_r(rot_step * (inputs[3] / stick_limit))
        rotation_r = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        ## right stick - right arm x
        euler = list(T.euler_from_quaternion(rotation_r))
        euler[1] -= limit_val_r(rot_step * (inputs[4] / stick_limit))
        rotation_r = T.quaternion_from_euler(euler[0], euler[1], euler[2])


        ## right button - right arm up
        if inputs[14] == 1:
            position_r[2] += pos_step

        ## right trigger - right arm down
        if inputs[15] > 700:
            position_r[2] -= pos_step

        ## X/B Button - right arm left/right
        if inputs[8] > 0:
            position_r[1] += pos_step
        if inputs[10] > 0:
            position_r[1] -= pos_step

        ## Y/A - right arm forward/back
        if inputs[11] > 0:
            position_r[0] += pos_step
        if inputs[9] > 0:
            position_r[0] -= pos_step


        if inputs[17] > 0:
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[0] -= rot_step
            rotation_r = T.quaternion_from_euler(euler[0], euler[1], euler[2])

    ## Mimicked control for both arms
    elif control_mode == 3:
        ## Mimicked control for both arms

        ## left stick - left arm y
        position_l[1] -= limit_val(pos_step * (inputs[0] / stick_limit))
        position_r[1] -= limit_val(pos_step * (inputs[0] / stick_limit))


        ## left stick - left arm x
        position_l[0] -= limit_val(pos_step * (inputs[1] / stick_limit))
        position_r[0] -= limit_val(pos_step * (inputs[1] / stick_limit))


        ## left button - left arm up
        if inputs[12] == 1:
            position_l[2] += pos_step
            position_r[2] += pos_step


        ## left trigger - left arm down
        if inputs[13] > 700:
            position_l[2] -= pos_step
            position_r[2] -= pos_step


        ## Dpad left/right- left arm rotation left/right
        if inputs[6] > 0:
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[2] -= rot_step
            rotation_l = T.quaternion_from_euler(euler[0], euler[1], euler[2])
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[2] -= rot_step
            rotation_r = T.quaternion_from_euler(euler[0], euler[1], euler[2])
        elif inputs[6] < 0:
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[2] += rot_step
            rotation_l = T.quaternion_from_euler(euler[0], euler[1], euler[2])
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[2] += rot_step
            rotation_r = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        ## Dpad up/down - left arm rotation up/down
        if inputs[7] > 0:
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[1] += rot_step
            rotation_l = T.quaternion_from_euler(euler[0], euler[1], euler[2])
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[1] += rot_step
            rotation_r = T.quaternion_from_euler(euler[0], euler[1], euler[2])
        elif inputs[7] < 0:
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[1] -= rot_step
            rotation_l = T.quaternion_from_euler(euler[0], euler[1], euler[2])
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[1] -= rot_step
            rotation_r = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        if inputs[16] > 0:
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[0] += rot_step
            rotation_l = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        ## right stick - right arm y
        position_r[1] -= limit_val(pos_step * (inputs[3] / stick_limit))

        ## right stick - right arm x
        position_r[0] -= limit_val(pos_step * (inputs[4] / stick_limit))

        ## right button - right arm up
        if inputs[14] == 1:
            position_r[2] += pos_step

        ## right trigger - right arm down
        if inputs[15] > 700:
            position_r[2] -= pos_step

        ## X/B Button - right arm rotation
        if inputs[8] > 0:
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[2] += rot_step
            rotation_r = T.quaternion_from_euler(euler[0], euler[1], euler[2])
        if inputs[10] > 0:
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[2] -= rot_step
            rotation_r = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        ## Y/A - right arm rotation
        if inputs[11] > 0:
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[1] -= rot_step
            rotation_r = T.quaternion_from_euler(euler[0], euler[1], euler[2])
        if inputs[9] > 0:
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[1] += rot_step
            rotation_r = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        if inputs[17] > 0:
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[0] -= rot_step
            rotation_r = T.quaternion_from_euler(euler[0], euler[1], euler[2])

    ## One arm - plane and rotation on sticks
    elif control_mode == 4:
        if inputs[12] == 1:
            control_side = 0
            print("Controlling left arm")
        if inputs[14] == 1:
            control_side = 1
            print("Controlling right arm")
        if control_side == 0:
            position = position_l
            rotation = rotation_l
        else:
            position = position_r
            rotation = rotation_r

        ## left stick - arm y - left/right
        position[1] -= limit_val(pos_step * (inputs[0] / stick_limit))

        ## left stick - arm x - forward/back
        position[0] -= limit_val(pos_step * (inputs[1] / stick_limit))

        ## right stick - arm rotation y
        euler = list(T.euler_from_quaternion(rotation))
        euler[2] += limit_val_r(rot_step * (inputs[3] / stick_limit))
        rotation = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        ## right stick - arm rotation x
        euler = list(T.euler_from_quaternion(rotation))
        euler[1] -= limit_val_r(rot_step * (inputs[4] / stick_limit))
        rotation = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        ## Dpad left/right- arm rotation left/right
        if inputs[6] > 0:
            euler = list(T.euler_from_quaternion(rotation))
            euler[0] += rot_step
            rotation = T.quaternion_from_euler(euler[0], euler[1], euler[2])
        elif inputs[6] < 0:
            euler = list(T.euler_from_quaternion(rotation))
            euler[0] -= rot_step
            rotation = T.quaternion_from_euler(euler[0], euler[1], euler[2])

        ## Dpad up/down - height up/down
        if inputs[7] > 0:
            position[2] -= pos_step
        elif inputs[7] < 0:
            position[2] += pos_step

        if control_side == 0:
            position_l = position
            rotation_l = rotation
        else:
            position_r = position
            rotation_r = rotation


    if (control_mode == 5):
        ## Create and change transformation matrix
        transform_rot[0] += limit_val_r(rot_step * (inputs[3] / stick_limit))
        transform_rot[1] += limit_val_r(rot_step * (inputs[4] / stick_limit))
        rot_matrix = t3d.euler.euler2mat(transform_rot[0], transform_rot[1], transform_rot[2])
        transform = t3d.affines.compose([0, 0, 0], rot_matrix, [1, 1, 1])

        ## Send transform matrix
        transform_msg = Transform()
        transform_msg.translation.x = 0
        transform_msg.translation.y = 0
        transform_msg.translation.z = 0
        rot_quat = T.quaternion_from_euler(transform_rot[0], transform_rot[1], transform_rot[2])
        transform_msg.rotation.w = rot_quat[0]
        transform_msg.rotation.x = rot_quat[1]
        transform_msg.rotation.y = rot_quat[2]
        transform_msg.rotation.z = rot_quat[3]
        transform_pub.publish(transform_msg)


    ## Add initial tip to copSim
    ee_pose_goals = EEPoseGoals()
    pose_r = Pose()
    pose_r.position.x = position_r[0]
    pose_r.position.y = position_r[1]
    pose_r.position.z = position_r[2]

    pose_r.orientation.w = rotation_r[0]
    pose_r.orientation.x = rotation_r[1]
    pose_r.orientation.y = rotation_r[2]
    pose_r.orientation.z = rotation_r[3]

    pose_l = Pose()
    pose_l.position.x = position_l[0]
    pose_l.position.y = position_l[1]
    pose_l.position.z = position_l[2]

    if (control_mode == 5):
        ## Apply transformation matrix
        position_old = [position_l[0], position_l[1], position_l[2]]
        position_old.append(0.)
        # print("Old: ")
        # print(position_old)
        # print(transform)
        position_new = np.matmul(transform, position_old)
        # print("New: ")
        # print(position_new)
        pose_l = Pose()
        pose_l.position.x = position_new[0]
        pose_l.position.y = position_new[1]
        pose_l.position.z = position_new[2]

    pose_l.orientation.w = rotation_l[0]
    pose_l.orientation.x = rotation_l[1]
    pose_l.orientation.y = rotation_l[2]
    pose_l.orientation.z = rotation_l[3]
    ee_pose_goals.ee_poses.append(pose_r)
    ee_pose_goals.ee_poses.append(pose_l)
    # print(pose_l)
    # print(pose_r)
    ee_pose_goals_pub.publish(ee_pose_goals)

    r.sleep()


