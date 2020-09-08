#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose
from std_msgs.msg import Bool
from relaxed_ik.msg import EEPoseGoals
import RelaxedIK.Utils.transformations as T
from inputs import get_gamepad
from movo_mimicry.msg import XboxInputs


# ---
# Gets Xbox controller inputs and outputs onto ROS topics
# ---

rospy.init_node('xbox_to_ros')

xbox_pub = rospy.Publisher('/xbox_inputs', XboxInputs, queue_size=5)

codes = {"ABS_X": 0,        # Left Stick Left/Right
         "ABS_Y": 1,        # Left Stick Up/Down
         "BTN_THUMBL": 2,   # Left Stick Click
         "ABS_RX": 3,       # Right Stick Left/Right
         "ABS_RY": 4,       # Right Stick Up/Down
         "BTN_THUMBR": 5,   # Right Stick Click
         "ABS_HAT0X": 6,    # D-Pad Left/Right
         "ABS_HAT0Y": 7,    # D-Pad Up/Down
         "BTN_NORTH": 8,    # X Button
         "BTN_SOUTH": 9,    # A Button
         "BTN_EAST": 10,    # B Button
         "BTN_WEST": 11,    # Y Button
         "BTN_TL": 12,      # Left Button (LB)
         "ABS_Z": 13,       # Left Trigger (LT)
         "BTN_TR": 14,      # Right Button (RB)
         "ABS_RZ": 15,      # Right Trigger (RT)
         "BTN_SELECT": 16,  # Select Button (Left Side)
         "BTN_START": 17,   # Start Button (Right Side)
         "BTN_MODE": 18}    # Xbox Button
inputs = [0] * 19

while not rospy.is_shutdown():
    events = get_gamepad()
    for event in events:
        if event.ev_type != "Sync":
            # print(event.ev_type, event.code, event.state)
            inputs[codes[event.code]] = event.state
    print(inputs)
    send_msg = XboxInputs()
    send_msg.inputs = inputs
    xbox_pub.publish(send_msg)




