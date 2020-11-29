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
global next_objects_secret_position
next_objects_secret_position = []
global robot_odom_position
robot_odom_position = []
global objects_odom_position
objects_odom_position = []
global next_objects_odom_position
next_objects_odom_position = []
global next_object_secret_position
next_object_secret_position = []
global object_odom_position
object_odom_position = []

global tags_1
tags_1 = []
global tags_2
tags_2 = []
global tag
tag = 0
global trans
trans = [0, 0, 0]
global rot
rot = [0, 0, 0, 0]
global code_status
code_status = 0

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
    global objects_odom_position
    global object_odom_position
    global tag
    global tags_2
    global trans
    global rot
    listener = TransformListener()
    listener.waitForTransform('/odom', '/camera_optical_link', rospy.Time(), rospy.Duration(4.0))
    now = rospy.Time.now()
    listener.waitForTransform('/odom', '/camera_optical_link', now, rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('/odom', '/camera_optical_link', now)
    if qr_code_info != []:
        if(qr_code_info[4] not in tags_2):
            if(np.size(tags_2)<=2 or (np.size(tags_2)>2 and (float(qr_code_info[4])-float(tag)==1 or float(qr_code_info[4])-float(tag)==-4))):
                try:
                    (r_cam, p_cam, y_cam) = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
                    trans_rotation_cam_odom = np.insert(eulerAnglesToRotationMatrix(r_cam, p_cam, y_cam), 3, [0, 0, 0], 0)
                    trans_translation_cam_odom = [trans[0], trans[1], trans[2], 1]
                    trans_cam_odom = np.insert(trans_rotation_cam_odom, 3, trans_translation_cam_odom, 1)
                    object_odom_position = np.dot(trans_cam_odom, [message.pose.position.x, message.pose.position.y, message.pose.position.z, 1]) 
                    objects_odom_position.append(object_odom_position)

                    tags_2.append(tag)
                    print("Current tag: ", tag)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    print("fault")
            
def code_message_callback(message):
    global qr_code_info
    global objects_secret_position
    global next_objects_secret_position
    global next_object_secret_position
    global tag
    global tags_1
    global trans
    global rot
    qr_code_info = re.findall(r"[-+]?\d*\.\d+|\d+", str(message))
    
    if qr_code_info != []:
        if qr_code_info[4] not in tags_1:            
            if(np.size(tags_1)<=2 or (np.size(tags_1)>2 and (float(qr_code_info[4])-float(tag)==1 or float(qr_code_info[4])-float(tag)==-4))):
                tag = qr_code_info[4]
                tags_1.append(tag)
                objects_secret_position.append([qr_code_info[0], qr_code_info[1]])
                next_object_secret_position = [qr_code_info[2], qr_code_info[3]]
                next_objects_secret_position.append(next_object_secret_position)

 
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

if __name__ == '__main__':
    rospy.init_node('patrol')
    rate = rospy.Rate(10.0)
    trans_secret_odom_solved = False    # if the transform matrix from secret to odom solved

    #rospy.Subscriber("/odom", Odometry, odom_callback)
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rospy.Subscriber("visp_auto_tracker/status", Int8, status_callback, queue_size=10)
    rospy.Subscriber("visp_auto_tracker/code_message", String, code_message_callback, queue_size=10)
    rospy.Subscriber("visp_auto_tracker/object_position", PoseStamped, object_position_callback, queue_size=10)

    while not trans_secret_odom_solved:
        # if np.size(tags_1)==2 and np.size(tags_2)!=2:
        #     twist = Twist()
        #     twist.linear.x = 0.0
        #     twist.angular.z = 0.0
        #     cmd_vel.publish(twist)
        #     print("tags_1 = 2, tags_2 != 2")
        #     rospy.sleep(3)
        if np.size(tags_1)==2 and np.size(tags_2)==2:
            trans_secret_odom_solved = True
            print("The transform from secret frame to odom frame is: ")
            trans_secret_odom = trans_secret_odom(objects_odom_position, objects_secret_position)
            print(trans_secret_odom)

    while trans_secret_odom_solved:
        print("next_object_secret_position", next_object_secret_position)
        next_robot_secret_position = [float(next_object_secret_position[0]), float(next_object_secret_position[1])]
        next_robot_odom_position = np.dot(trans_secret_odom, np.array(np.insert(next_robot_secret_position, 2, [0, 1], 0), dtype='float64'))
        next_robot_odom_position = [next_robot_odom_position, [0, 0, 0, 1]]
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        
        goal_position = goal_pose([object_odom_position,[0,0,0,1]])
        client.send_goal(goal_position)
        client.wait_for_result()
        rospy.sleep(5)
        
        if np.size(tags_1) == 5 and np.size(tags_2) == 5:
            break
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
