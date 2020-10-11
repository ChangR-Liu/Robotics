#!/usr/bin/env python

import rospy, tf, random
import tf_conversions
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
import os

if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    print("Got it.")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    with open("/home/crliu/catkin_ws/src/hello_ros/urdf/cube.urdf", "r") as f:
        product_xml = f.read()

    orient = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, 0.785398))

    num_of_cubes = random.randint(2,6)

    for num in xrange(0,num_of_cubes):
        bin_y   =   random.uniform(0,0.5)
        bin_x   =   random.uniform(0,0.5)
        item_name   =   "cube{}".format(num)
        print("Spawning model:%s", item_name)
        item_pose   =   Pose(Point(x=bin_x, y=bin_y,    z=1),   orient)
        spawn_model(item_name, product_xml, "", item_pose, "world")

    with open("/home/crliu/catkin_ws/src/hello_ros/urdf/bucket.urdf", "r") as f:
        product_xml = f.read()

    item_pose   =   Pose(Point(x=0.53, y=-0.23,    z=0.78),   orient)
    print("Spawning model:%s", "bucket")
    spawn_model("bucket", product_xml, "", item_pose, "world")

    # #Read the position
    # get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    # model = GetModelStateRequest()
    # for num in xrange(0,num_of_cubes):
    #     model.model_name =  "cube{}".format(num)
    #     objstate = get_state_service(model)
    #     objx=objstate.pose.position.x
    #     objy=objstate.pose.position.y
    #     objz=objstate.pose.position.z
    #     #print(objx)
    #     os.system('python /home/crliu/catkin_ws/src/hello_ros/scripts/mini_project.py {} {} {}'.format(float(objx),float(objy),float(objz)))