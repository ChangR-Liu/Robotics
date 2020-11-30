#!/usr/bin/env python

import numpy as np
import rospy
import actionlib
import nav_msgs.msg
import re
import tf
import math
import copy

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int8
from geometry_msgs.msg import PoseStamped, Twist
from scipy.optimize import root, fsolve
from tf.listener import TransformListener
from tf.transformations import quaternion_from_euler
"""
waypoints = [
    [(1.13, -1.6, 0.0), (0.0, 0.0, -0.16547, -0.986213798314)],
    [(0.81, 0.32, 0.0), (0.0, 0.0, -0.99, 0.0333)]
]
"""

global qr_code_info
qr_code_info = []

global objects_secret_position
objects_secret_position = []
global object_secret_position
object_secret_position = []

global next_objects_secret_position
next_objects_secret_position = []

global robot_odom_position
robot_odom_position = []

global objects_odom_position
objects_odom_position = []
global object_odom_position
object_odom_position = []

global next_objects_odom_position
next_objects_odom_position = []
global next_object_secret_position
next_object_secret_position = []

global SMaRt_element
SMaRt_element = ''
global SMaRt_messages
SMaRt_messages = {}


global tags
tags = ['0']
global tag
tag = '0'
global code_status
code_status = 0

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

def status_callback(message):
    global code_status
    code_status=message.data

def odom_callback(message):
    global robot_odom_position
    robot_odom_position = nav_msgs.msg.Odometry()
    robot_odom_position = message
    br = tf.TransformBroadcaster()
    br.sendTransform((robot_odom_position.pose.pose.position.x, 
        robot_odom_position.pose.pose.position.y, 0), 
        tf.transformations.quaternion_from_euler(0, 0, 0), 
        rospy.Time.now(), 
        "robot_map_position", 
        "odom")

def trans_secret_odom(objects_odom_position, objects_secret_position):
    #print(objects_odom_position)
    #print(objects_secret_position)
    """
    A = np.array([[float(objects_secret_position[0][0]), -float(objects_secret_position[0][1]), 1, 0],
            [float(objects_secret_position[0][1]), float(objects_secret_position[0][0]), 0, 1],
            [float(objects_secret_position[1][0]), -float(objects_secret_position[1][1]), 1, 0],
            [float(objects_secret_position[1][1]), float(objects_secret_position[1][0]), 0, 1]])

    b = np.array([[objects_odom_position[0][0], objects_odom_position[0][1], objects_odom_position[1][0], objects_odom_position[1][1]]]).T

    results = np.linalg.solve(A, b)
    """
    sub_x_world_12 = float(objects_odom_position[0][0]) - float(objects_odom_position[1][0])
    sub_y_world_12 = float(objects_odom_position[0][1]) - float(objects_odom_position[1][1])
    sub_x_secret_12 = float(objects_secret_position[0][0]) - float(objects_secret_position[1][0])
    sub_y_secret_12 = float(objects_secret_position[0][1]) - float(objects_secret_position[1][1])

    sin_theta = (sub_x_secret_12 * sub_y_world_12 - sub_y_secret_12 * sub_x_world_12) / (pow(sub_x_secret_12, 2) + pow(sub_y_secret_12, 2))
    cos_theta = (sub_x_world_12 + sin_theta * sub_y_secret_12) / sub_x_secret_12
    theta = math.atan2(sin_theta, cos_theta)
    x_ref_world = float(objects_odom_position[1][0]) - cos_theta * float(objects_secret_position[1][0]) + sin_theta * float(objects_secret_position[1][1])
    y_ref_world = float(objects_odom_position[1][1]) - sin_theta * float(objects_secret_position[1][0])- cos_theta * float(objects_secret_position[1][1])
    results = [cos_theta, sin_theta, x_ref_world, y_ref_world]

    T = np.array([[results[0], -results[1], 0, results[2]], 
            [results[1], results[0], 0, results[3]], 
            [0, 0, 1, 0],
            [0, 0, 0, 1]], dtype='float64')
    return T
 
def eulerAnglesToRotationMatrix(r, p, y):
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(r), -math.sin(r)],
                    [0, math.sin(r), math.cos(r)]])

    R_y = np.array([[math.cos(p), 0, math.sin(p)],
                    [0, 1, 0],
                    [-math.sin(p), 0, math.cos(p)]])

    R_z = np.array([[math.cos(y), -math.sin(y), 0],
                    [math.sin(y), math.cos(y), 0],
                    [0, 0, 1]])
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

def object_position_callback(message):
    global object_camera_position
    object_camera_position = [message.pose.position.x, message.pose.position.y, message.pose.position.z, 1]
            
def code_message_callback(message):
    global qr_code_info
    global object_secret_position
    global next_object_secret_position
    global tag
    global SMaRt_element
    qr_code_info = re.findall(r"[-+]?\d*\.\d+|\d+", str(message))
    if qr_code_info != []:
        object_secret_position = [qr_code_info[0], qr_code_info[1], 0, 1]
        next_object_secret_position = [qr_code_info[2], qr_code_info[3], 0, 1]
        tag = qr_code_info[4]
        SMaRt_element = str(message)[-2]

if __name__ == '__main__':
    rospy.init_node('patrol')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    rate = rospy.Rate(10)
    #rospy.Subscriber("/odom", Odometry, odom_callback)
    Cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    listener = TransformListener()
    rospy.Subscriber("visp_auto_tracker/status", Int8, status_callback, queue_size=10)
    rospy.Subscriber("visp_auto_tracker/code_message", String, code_message_callback, queue_size=10)
    rospy.Subscriber("visp_auto_tracker/object_position", PoseStamped, object_position_callback, queue_size=10)

    goal_position = goal_pose([[-4, 0, 0], [0, 0, 0, 0]])
    client.send_goal(goal_position)
    client.wait_for_result()

    while(np.size(tags)<3):
        object_odom_position_solved = False
        twist_1 = Twist()
        twist_1.angular.z = 0.1
        Cmd_vel.publish(twist_1)
        while not object_odom_position_solved:
            if tag not in tags:
                print("tag:",tag)
                tags.append(tag)
                SMaRt_messages.update({int(tag): SMaRt_element})
                twist_2 = Twist()
                Cmd_vel.publish(twist_2)
                rospy.sleep(3)
                try:
                    # obtatin object_odom_position
                    (trans, rot) = listener.lookupTransform('/odom', '/camera_optical_link', rospy.Time(0))
                    (r_cam, p_cam, y_cam) = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
                    trans_rotation_cam_odom = np.insert(eulerAnglesToRotationMatrix(r_cam, p_cam, y_cam), 3, [0, 0, 0], 0)
                    trans_translation_cam_odom = [trans[0], trans[1], trans[2], 1]
                    trans_cam_odom = np.insert(trans_rotation_cam_odom, 3, trans_translation_cam_odom, 1)

                    object_odom_position = np.dot(trans_cam_odom, object_camera_position)
                    objects_odom_position.append(object_odom_position)

                    objects_secret_position.append(object_secret_position)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
            object_odom_position_solved = True
            # print("object_odom_position: ", object_odom_position)
        if np.size(tags)==3:
            # print("objects_odom_position: ", objects_odom_position)
            # print("objects_secret_position: ", objects_secret_position)
            trans_secret_odom = trans_secret_odom(objects_odom_position, objects_secret_position)
            print("The transform from secret frame to odom frame is: ")
            print(trans_secret_odom)

    while object_odom_position_solved:
        print(SMaRt_messages)
        if np.size(tags) == 6:
            break
        print("next_object_secret_position", next_object_secret_position)
        goal_tag = tag
        print("tag: ", tag)
        next_robot_secret_position = [float(next_object_secret_position[0])*0.75, float(next_object_secret_position[1])*0.75]
        next_robot_odom_position = np.dot(trans_secret_odom, np.array(np.insert(next_robot_secret_position, 2, [0, 1], 0), dtype='float64'))
        next_robot_odom_position = [next_robot_odom_position, [0,0,0,1]]

        
        print("next_robot_odom_position", next_robot_odom_position)
        goal_position = goal_pose(next_robot_odom_position)
        client.send_goal(goal_position)
        client.wait_for_result()
        rospy.sleep(5)
        if int(tag) - int(goal_tag) == 1 or int(tag) - int(goal_tag) == -4:
            SMaRt_messages.update({int(tag): SMaRt_element})
            tags.append(tag)
            continue
        else:
            while(int(tag) - int(goal_tag) != 1 and int(tag) - int(goal_tag) != -4):
                twist_3 = Twist()
                twist_3.angular.z = 0.2
                Cmd_vel.publish(twist_3)
            SMaRt_messages.update({int(tag): SMaRt_element})
            tags.append(tag)
            continue
        
    SMaRt_messages = sorted(SMaRt_messages.items(), key=lambda x:x[0])
    print(SMaRt_messages)
"""
        next_point = [(robot_map_position.pose.pose.position.x, robot_map_position.pose.pose.position.y, 0), (0, 0, 0, 1)]
        goal_rotation = goal_pose(next_point)
        client.send_goal(goal_rotation)
        client.wait_for_result()
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
