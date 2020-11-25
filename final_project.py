#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
import re
import tf
"""
waypoints = [
    [(1.13, -1.6, 0.0), (0.0, 0.0, -0.16547, -0.986213798314)],
    [(0.81, 0.32, 0.0), (0.0, 0.0, -0.99, 0.0333)]
]
"""
tag = 10
tags = []
x_next = 0
y_next = 0
next_point = []

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

def qr_code(code_message):
    global tag, tags, x_next, y_next, next_point
    qr_code_info = re.findall(r"\d+\.?\d*", str(code_message))



    if qr_code_info != []:
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

if __name__ == '__main__':
    rospy.init_node('patrol')
    flag1=True
    while True:
        if flag1==True:
            rospy.Subscriber("visp_auto_tracker/code_message", String, qr_code, queue_size=1)
            flag1=False;
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        goal = goal_pose(next_point)
        client.send_goal(goal)
        client.wait_for_result()
        next_point = [(next_point[0][0], next_point[0][1], next_point[0][2]), (0, 0, 0, 1)]
        goal_rotation = goal_pose(next_point)
        client.send_goal(goal_rotation)
        client.wait_for_result()
        flag1=True;
        rospy.sleep(3)