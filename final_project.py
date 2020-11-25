#!/usr/bin/env python

import numpy as np
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import re
import tf
from scipy.optimize import root
from tf.listener import TransformListener
from tf.transformations import quaternion_from_euler
"""
waypoints = [
    [(1.13, -1.6, 0.0), (0.0, 0.0, -0.16547, -0.986213798314)],
    [(0.81, 0.32, 0.0), (0.0, 0.0, -0.99, 0.0333)]
]
"""
tag = 0
tags = []
x_next = 0
y_next = 0
next_point = []

global qr_maps
qr_maps = []
global secret_position
secret_position = []
global qr_code_info
qr_code_info = []
global next_secret_position
next_secret_position = []


def secret2map(qr_map, secret_position):
    def f(x):
        eqs = []
        eqs.append(-qr_map[0][0]+secret_position[0][0]*np.cos(x[0])-secret_position[0][1]*np.sin(x[0])+x[1])
        eqs.append(-qr_map[0][1]+secret_position[0][0]*np.sin(x[0])+secret_position[0][1]*np.cos(x[0])+x[2])
        eqs.append(-qr_map[1][0]+secret_position[1][0]*np.cos(x[0])-secret_position[1][1]*np.sin(x[0])+x[1])
        return eqs
    results = root(f, [0, 0, 0]).x
    T = np.array([np.cos(results[0]), -np.sin(results[0]), 0, results[1]], 
            [np.sin(results[0]), np.cos(results[0]), 0, results[2]], 
            [0, 0, 1, 0],
            [0, 0, 0, 1])
    return T



def goal_pose(pose):
    if pose == []:
        goal_pose = []
        return goal_pose
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    return goal_pose

def object_position_callback(message):
    listener = TransformListener()
    if qr_code_info != []:
        try:
            (trans, rot) = listener.lookupTransform('/odom', '/camera_optical_link', rospy.Time(0))
            print(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("fault")
    #if qr_code_info != []: 
        (r_cam, p_cam, y_cam) = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
        trans_euler_odom_cam = eulerAnglesToRotationMatrix([r_cam, p_cam, y_cam])
        trans_matrix_R2M = np.insert(np.insert(trans_euler_odom_cam, 3, [0, 0, 0], 0), 3,
                                        [trans[0], trans[1], trans[2], 1], 1)
        qr_map = np.dot(trans_matrix_R2M, [message.pose.position.x, message.pose.position.y, message.pose.position.z, 1]) 
        qr_maps.append(qr_map)
"""
    if qr_code_info != [] and tag != 0:
        tag = qr_code_info[4]
        tags.append(tag)
        x_next = qr_code_info[2]
        y_next = qr_code_info[3]
        next_point = [(float(y_next)-6,-float(x_next), 0), tf.transformations.quaternion_from_euler(0 , 0, 0)]
        print("next_point: ", next_point, "current_tag: ", tag)
        goal = goal_pose(next_point)
        client.send_goal(goal)
        client.wait_for_result()
        # next_point = [(next_point[0][0], next_point[0][1], next_point[0][2]), (0, 0, 0, 1)]
        # goal_rotation = goal_pose(next_point)
        # client.send_goal(goal_rotation)
        # client.wait_for_result()
"""

def code_message_callback(message):
    global qr_code_info
    global secret_position
    global next_secret_position
    qr_code_info = re.findall(r"\d+\.?\d*", str(message))
    if(qr_code_info != []):
        secret_position.append([qr_code_info[0], qr_code_info[1]])
        next_secret_position.append([qr_code_info[2], qr_code_info[3]])

if __name__ == '__main__':
    rospy.init_node('patrol')
    while True:
        rospy.Subscriber("visp_auto_tracker/object_position", PoseStamped, object_position_callback, queue_size=1)
        rospy.Subscriber("visp_auto_tracker/code_message", String, code_message_callback, queue_size=1)

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        next_point = [(-4, 0, 0), (0, 0, 0, 0)]
        goal_rotation = goal_pose(next_point)
        client.send_goal(goal_rotation)
        client.wait_for_result()
        next_point = [(-4, 0, 0), (0, 0, 0, 1)]
        goal_rotation = goal_pose(next_point)
        client.send_goal(goal_rotation)
        client.wait_for_result()

"""
        print(qr_maps)
        print(secret_position)
        print(qr_code_info)
        print(next_secret_position)
"""

"""
        goal = goal_pose(next_point)
        client.send_goal(goal)
        client.wait_for_result()
        next_point = [(next_point[0][0], next_point[0][1], next_point[0][2]), (0, 0, 0, 1)]
        goal_rotation = goal_pose(next_point)
        client.send_goal(goal_rotation)
        client.wait_for_result()
        flag1=True;
        rospy.sleep(3)
"""
