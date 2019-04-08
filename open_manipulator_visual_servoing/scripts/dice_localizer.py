#!/usr/bin/env python

import rospy
from open_manipulator_visual_servoing.srv import *
import numpy
import tf
import math
from geometry_msgs.msg import Quaternion
import moveit_commander
import moveit_msgs.msg
import sensor_msgs.msg
from std_msgs.msg import Header

def move_to_named_pose(name):
   arm_group.clear_pose_targets()
   
   joint_state = sensor_msgs.msg.JointState()
   joint_state.header = Header()
   joint_state.header.stamp = rospy.Time.now()
   joint_state.position = arm_group.get_current_joint_values()
   moveit_robot_state = moveit_msgs.msg.RobotState()
   moveit_robot_state.joint_state = joint_state
   arm_group.set_start_state(moveit_robot_state)
   
   arm_group.set_named_target(name)
   plan = arm_group.plan()
   
   arm_group.execute(plan, wait=True)
    
#subscribes to the depth finding service   
def indexer_client(u, v):
    rospy.wait_for_service('visual_servoing/depth_cloud/indexer')
    try:
        indexer = rospy.ServiceProxy('visual_servoing/depth_cloud/indexer', IndexPointCloud)
        resp = indexer(u, v)
        return resp.point.x, resp.point.y, resp.point.z
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
                
#subscribes to the blob finding service
def blob_client(colors):
    rospy.wait_for_service('visual_servoing/color/blob_finder')
    try:
        blob_finder = rospy.ServiceProxy('visual_servoing/color/blob_finder', LocateBlob)
        resp = blob_finder(colors)
        return resp.x, resp.y
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def p():
    print "UVs:"
    print red_uv
    print green_uv
    print blue_uv
    print 
    print "Points:"
    print red_xyz
    print green_xyz
    print blue_xyz
    
rospy.init_node("kinect_localizer_broadcaster")
    
arm_group = moveit_commander.MoveGroupCommander('arm')

move_to_named_pose("start")
#reads in red die location                
red_range = [0, 0, 80, 60, 30, 210]
red_uv = blob_client(red_range)
red_xyz = indexer_client(red_uv[0], red_uv[1])

#reads in green die location
green_range = [40, 80, 0, 120, 255, 50]
green_uv = blob_client(green_range)
green_xyz = indexer_client(green_uv[0], green_uv[1])

#reads in blue die location
blue_range = [150, 50, 0, 255, 150, 50]
blue_uv = blob_client(blue_range)
blue_xyz = indexer_client(blue_uv[0], blue_uv[1])

#creates a point for the base of the arm
center = numpy.divide(numpy.add(red_xyz, green_xyz), 2)
base = numpy.add(center, numpy.subtract(center, blue_xyz))

#creates vectors from points
forward_vector = numpy.subtract(green_xyz, base)
right_vector = numpy.subtract(red_xyz, base)
up_vector = numpy.cross(right_vector, forward_vector)

#normalizes vectors
forward_vector = tf.transformations.unit_vector(forward_vector)
right_vector = tf.transformations.unit_vector(right_vector)
up_vector = tf.transformations.unit_vector(up_vector)

#Creating a matrix from vectors
vector_matrix = numpy.array((right_vector,forward_vector,up_vector))

#Inverse matrix to create relationship between camera and arm
matrix_inverse = tf.transformations.inverse_matrix(vector_matrix)

#Creates Euler from Inverted Matrix, This is because tf.transformations.quaternion_from_matrix didn't like the sizing of the matrix, so an Euler array was created to bypass this.
Euler = tf.transformations.euler_from_matrix(matrix_inverse)

#Creates Quaternions from Euler
q = tf.transformations.quaternion_from_euler(Euler[0],Euler[1],Euler[2])

#Prints out data, used to debug
p()

br = tf.TransformBroadcaster()
rate = rospy.Rate(10.0)

while not rospy.is_shutdown():
   br.sendTransform(base, q, rospy.Time.now(), "localizer_frame", "kinect_point_frame")
   br.sendTransform([0, 0, 0], [0, 0, 0, 1], rospy.Time.now(), "base_link", "localizer_frame")
   br.sendTransform([0, 0, 0], [0, 0, 0, 1], rospy.Time.now(), "world", "localizer_frame")
   rate.sleep()