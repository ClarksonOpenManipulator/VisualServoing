#!/usr/bin/env python

import rospy
import copy
from geometry_msgs.msg import *
from open_manipulator_visual_servoing.srv import *
from open_manipulator_moveit_config.srv import *
import tf
import math
import numpy

#subscribes to the object checking service   
def checker_client(colors):
    rospy.wait_for_service('visual_servoing/object/checker')
    try:
        checker = rospy.ServiceProxy('visual_servoing/object/checker', ObjectChecking)
        resp = checker(colors)
        return resp.object_value
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
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
        
#subscribes to the move_group_commander service
def move_robot(target):
    rospy.wait_for_service('open_manipulator/moveit/arm/commander')
    try:
        move_robot = rospy.ServiceProxy('open_manipulator/moveit/arm/commander', MoveGroupCommand)
        resp = move_robot(target)
        return resp.planSuccess and resp.executeSuccess
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def grip(onOff):
    global gripper
    gripper.publish(onOff)
    rospy.sleep(1)
    
def search(fixed): 
    global listener
    red_range = [0, 0, 80, 60, 30, 210]
    green_range = [40, 80, 0, 120, 255, 50]
    blue_range = [150, 50, 0, 255, 150, 50]

    color_ranges = [red_range, green_range, blue_range]

    move_robot(fixed)

    dice_map = {}
    for colors in color_ranges:
        cur_uv = blob_client(colors)
        cur_xyz = indexer_client(cur_uv[0], cur_uv[1])
        
        red_range = [0, 0, 70, 120, 60, 255]
        green_range = [50, 60, 0, 180, 255, 30]
        blue_range = [140, 50, 0, 255, 150, 70]

        color_ranges = [red_range, green_range, blue_range]

        cur_point = PoseStamped();
        cur_point.header.frame_id = "kinect_point_frame"
        cur_point.header.stamp = rospy.Time(0)
        cur_point.pose.position.x = cur_xyz[0]
        cur_point.pose.position.y = cur_xyz[1]
        cur_point.pose.position.z = cur_xyz[2]

        cur_point = listener.transformPose("localizer_frame", cur_point)
        
        xyFactor = -0.05
        zOffset = 0.0625
        zLift = 0.1
        
        posVector = [cur_point.pose.position.x, cur_point.pose.position.y, cur_point.pose.position.z]
        offsetVector = tf.transformations.unit_vector(posVector)
        offsetVector[0] *= xyFactor
        offsetVector[1] *= xyFactor
        offsetVector[2] = zOffset;
        
        posVector = numpy.add(posVector, offsetVector)
        cur_point.pose.position.x = posVector[0]
        cur_point.pose.position.y = posVector[1]
        cur_point.pose.position.z = posVector[2]
     
        o = cur_point.pose.position.x
        a = -cur_point.pose.position.y
        
        roll = 0
        pitch = math.radians(-180)
        yaw = math.atan2(o, a) 
        
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes='sxyz')
        cur_point.pose.orientation.x = q[0]
        cur_point.pose.orientation.y = q[1]
        cur_point.pose.orientation.z = q[2]
        cur_point.pose.orientation.w = q[3]
        
        cur_point.pose.position.z += zLift
        move_robot(cur_point.pose)

        cur_point.pose.position.z -= zLift
        move_robot(cur_point.pose)
        
        rospy.sleep(1)
        cur_value = checker_client(colors)
        rospy.sleep(1)
        
        cur_point.pose.position.z += zLift
        move_robot(cur_point.pose)

        cur_point.pose.position.z -= zLift
        
        posVector = numpy.subtract(posVector, offsetVector)
        cur_point.pose.position.x = posVector[0]
        cur_point.pose.position.y = posVector[1]
        cur_point.pose.position.z = posVector[2]
      
        if cur_value in dice_map:
            dice_map[cur_value].append(cur_point)
        else:
            dice_map[cur_value] = [cur_point]
   
    return dice_map

def test_search(fixed):
    global listener
    red_range = [0, 0, 80, 60, 30, 210]
    green_range = [40, 80, 0, 120, 255, 50]
    blue_range = [150, 50, 0, 255, 150, 50]

    color_ranges = [red_range, green_range, blue_range]
    numbers = [5, 4, 3]
    
    dice_map = {}
    for colors, num in zip(color_ranges, numbers):
        cur_uv = blob_client(colors)
        cur_xyz = indexer_client(cur_uv[0], cur_uv[1])

        cur_point = PoseStamped();
        cur_point.header.frame_id = "kinect_point_frame"
        cur_point.header.stamp = rospy.Time(0)
        cur_point.pose.position.x = cur_xyz[0]
        cur_point.pose.position.y = cur_xyz[1]
        cur_point.pose.position.z = cur_xyz[2]

        cur_point = listener.transformPose("localizer_frame", cur_point)
        
        cur_point.pose.orientation.x = 1
        cur_point.pose.orientation.y = 0
        cur_point.pose.orientation.z = 0
        cur_point.pose.orientation.w = 0
        
        if num in dice_map:
            dice_map[num].append(cur_point)
        else:
            dice_map[num] = [cur_point]
        
    return dice_map

def sort(fixed, dice_map):
    set_pose = Pose()
    set_pose.position.x = -0.1
    set_pose.position.y = 0.15
    set_pose.position.z = 0;
    
    set_pose.orientation.x = 1
    set_pose.orientation.y = 0
    set_pose.orientation.z = 0
    set_pose.orientation.w = 0

    
    set_spacing = 0.05
    
    xyFactor = 0.025
    zOffset = 0.01
    zLift = 0.1
    
    grip("grip_off")
    for key in sorted(dice_map.keys()):
        for value in dice_map[key]:
            
            posVector = [value.pose.position.x, value.pose.position.y, value.pose.position.z]
            offsetVector = tf.transformations.unit_vector(posVector)
            offsetVector[0] *= xyFactor
            offsetVector[1] *= xyFactor
            offsetVector[2] = zOffset;

            posVector = numpy.add(posVector, offsetVector)
            value.pose.position.x = posVector[0]
            value.pose.position.y = posVector[1]
            value.pose.position.z = posVector[2]
            
            value.pose.position.z += zLift
            move_robot(value.pose)
            
            value.pose.position.z -= zLift
            move_robot(value.pose)
            
            grip("grip_on")
            
            value.pose.position.z += zLift
            move_robot(value.pose)

            set_pose.position.z += zLift
            move_robot(set_pose)
            
            set_pose.position.z -= zLift
            move_robot(set_pose)
            grip("grip_off")
            
            set_pose.position.z += zLift
            move_robot(set_pose)
            move_robot(fixed)
            
            set_pose.position.z -= zLift
            set_pose.position.y += set_spacing

def main():
    global gripper
    global listener
    
    rospy.init_node('visual_servoing_dice_finder')
    listener = tf.TransformListener()
    gripper = rospy.Publisher('/gripper_command', std_msgs.msg.String, queue_size=1)
    listener.waitForTransform("/kinect_point_frame", "/localizer_frame", rospy.Time(0), rospy.Duration(20.0))

    fixed_rest = Pose()
    fixed_rest.position.x = 0.0
    fixed_rest.position.y = 0.2
    fixed_rest.position.z = 0.1
    fixed_rest.orientation.x = 1.0
    fixed_rest.orientation.y = 0.0
    fixed_rest.orientation.z = 0.0
    fixed_rest.orientation.w = 0.0

    data = search(fixed_rest)
    for key in data.keys():
        print "Number: " + str(key) + " Count: " + str(len(data[key]))
        
    sort(fixed_rest, data)

if __name__ == "__main__":
    main()