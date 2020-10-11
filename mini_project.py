#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
 
import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs
import math
import os
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from std_msgs.msg import String
 
def move_group_python_interface_tutorial():
  ## BEGIN_TUTORIAL
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)
 
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("Arm")
 
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)
 
  print "============ Starting tutorial "
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()
  ## We can also print the name of the end-effector link for this group
  print "============ End effector frame: %s" % group.get_end_effector_link()
  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()
  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"

  ## Let's setup the planner
  #group.set_planning_time(0.0)
  group.set_goal_orientation_tolerance(0.01)
  group.set_goal_tolerance(0.01)
  group.set_goal_joint_tolerance(0.01)
  group.set_num_planning_attempts(100)
  initial_value = group.get_current_joint_values()
  
  
 
  ## Read the cube positions
  get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
  model = GetModelStateRequest()
  for num in xrange(0,6):
    model.model_name =  "cube{}".format(num)
    objstate = get_state_service(model)
    objx=objstate.pose.position.x
    objy=objstate.pose.position.y
    objz=objstate.pose.position.z
    ## Planning to a Pose goal
    print "============ Generating plan 1"
    os.system('python /home/crliu/catkin_ws/src/hello_ros/scripts/lecture_5_4_open_gripper.py')
    waypoints = []
    group.go(initial_value,wait=True)
    pose_goal = group.get_current_pose().pose
    pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0,0.0,0.785398))
    # inisitial_posx = pose_goal.position.x
    # inisitial_posy = pose_goal.position.y
    # inisitial_posz = pose_goal.position.z
    # waypoints.append(pose_goal)
    pose_goal.position.x = objx
    pose_goal.position.y = objy
    pose_goal.position.z = objz + 0.3
    
    print pose_goal
    #Create waypoints
    waypoints.append(pose_goal)


    #createcartesian  plan
    (plan1, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
    plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)
   
   
    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(0.5)
   
   
    ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
    print "============ Visualizing plan1"
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory);
    print "============ Waiting while plan1 is visualized (again)..."
    rospy.sleep(2.)
   
    #If we're coming from another script we might want to remove the objects
    if "table" in scene.get_known_object_names():
      scene.remove_world_object("table")
    if "table2" in scene.get_known_object_names():
      scene.remove_world_object("table2")
    if "groundplane" in scene.get_known_object_names():
      scene.remove_world_object("groundplane")
   
    ## Moving to a pose goal
    group.execute(plan1,wait=True)
    rospy.sleep(2.)


   #  ## second movement
   #  rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.015)
    pose_goal = group.get_current_pose().pose
    # waypoints = []
    pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0,-1.5707963, 0.0))
   #  waypoints.append(pose_goal)
   # #  pose_goal2.position.x = inisitial_posx
   # #  pose_goal2.position.y = inisitial_posy
   # #  pose_goal2.position.z = inisitial_posz
   
   # #  #Create waypoints
   # #  waypoints2.append(pose_goal2)
   
   #  #createcartesian  plan
   #  (plan2, fraction) = group.compute_cartesian_path(
   #                                      waypoints,   # waypoints to follow
   #                                      0.01,        # eef_step
   #                                      0.0)         # jump_threshold
   #  plan2 = group.retime_trajectory(robot.get_current_state(), plan2, 1.0)
   #  rospy.sleep(4.)
   
   # ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
   #  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
   #  display_trajectory.trajectory_start = robot.get_current_state()
   #  display_trajectory.trajectory.append(plan2)
   #  display_trajectory_publisher.publish(display_trajectory);
   #  print "============ Waiting while plan2 is visualized (again)..."
   
   #  group.execute(plan2,wait=True)
   #  rospy.sleep(2.)
    group.go(pose_goal,wait=True)
    
     #  ## second movement
   #  rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.015)
    pose_goal = group.get_current_pose().pose
    waypoints = []
    waypoints.append(pose_goal)
    #pose_goal2.position.x = inisitial_posx
   #  pose_goal2.position.y = inisitial_posy
    pose_goal.position.z = objz+0.08
   
   #  #Create waypoints
    waypoints.append(pose_goal)
   
    #createcartesian  plan
    (plan2, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
    plan2 = group.retime_trajectory(robot.get_current_state(), plan2, 1.0)
    rospy.sleep(4.)
   
   ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan2)
    display_trajectory_publisher.publish(display_trajectory);
    print "============ Waiting while plan2 is visualized (again)..."
   
    group.execute(plan2,wait=True)
    rospy.sleep(2.)

    os.system('python /home/crliu/catkin_ws/src/hello_ros/scripts/lecture_5_4_close_gripper.py')
    rospy.sleep(4.)
    group.go(initial_value,wait=True)
    rospy.sleep(2.)
    #os.system('python /home/crliu/catkin_ws/src/hello_ros/scripts/lecture_5_4_open_gripper.py')

   
    ## When finished shut down moveit_commander.
    #moveit_commander.roscpp_shutdown()
   
    ## END_TUTORIAL
    print "============ STOPPING"
    #R = rospy.Rate(10)
    #while not rospy.is_shutdown():
    rospy.sleep(1)
      #R.sleep()
if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass

