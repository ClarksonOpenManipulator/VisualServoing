#!/usr/bin/env python

import rospy
import copy
from geometry_msgs.msg import *
from open_manipulator_visual_servoing.srv import *
from open_manipulator_moveit_config.srv import *
import tf

#subscribes to the object checking service   
def checker_client(check):
    rospy.wait_for_service('visual_servoing/object/checker')
    try:
        checker = rospy.ServiceProxy('visual_servoing/object/checker', ObjectChecking)
        resp = checker(check)
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

def main():
   global gripper
   rospy.init_node('visual_servoing_dice_finder')
   listener = tf.TransformListener()
   gripper = rospy.Publisher('/gripper_command', std_msgs.msg.String, queue_size=1)
   listener.waitForTransform("/kinect_point_frame", "/localizer_frame", rospy.Time(0), rospy.Duration(20.0))

   red_range = [0, 0, 80, 60, 30, 210]
   green_range = [40, 80, 0, 120, 255, 50]
   blue_range = [150, 50, 0, 255, 150, 50]
   
   color_ranges = [red_range, green_range, blue_range]
   
   fixed_rest = Pose()
   fixed_rest.position.x = 0.0
   fixed_rest.position.y = 0.2
   fixed_rest.position.z = 0.1
   fixed_rest.orientation.x = 1.0
   fixed_rest.orientation.y = 0.0
   fixed_rest.orientation.z = 0.0
   fixed_rest.orientation.w = 0.0
   
   fixed_1 = copy.deepcopy(fixed_rest)
   fixed_1.position.x = -0.1
   fixed_1.position.y = 0.15
   fixed_1.position.z = 0.0
   
   fixed_2 = copy.deepcopy(fixed_1)
   fixed_2.position.y += 0.05
   fixed_3 = copy.deepcopy(fixed_2)
   fixed_3.position.y += 0.05
   
   fixed_array = [fixed_1, fixed_2, fixed_3]
   
   for color, fixed in zip(color_ranges, fixed_array):
      cur_uv = blob_client(color)
      cur_xyz = indexer_client(cur_uv[0], cur_uv[1])
      
      cur_point = PoseStamped()
      cur_point.header.frame_id = "kinect_point_frame"
      cur_point.header.stamp = rospy.Time(0)
      
      cur_point.pose.position.x = cur_xyz[0]
      cur_point.pose.position.y = cur_xyz[1]
      cur_point.pose.position.z = cur_xyz[2] #- 0.0155
      
      cur_point = listener.transformPose("localizer_frame", cur_point)
      
      cur_point.pose.position.y += .01 # camera error correction
      
      cur_point.pose.orientation.x = 1
      cur_point.pose.orientation.y = 0
      cur_point.pose.orientation.z = 0
      cur_point.pose.orientation.w = 0
      
      move_robot(fixed_rest)
      grip("grip_off")
      
      cur_point.pose.position.z += 0.015 
      cur_point.pose.position.z += 0.1
      move_robot(cur_point.pose)
      rospy.sleep(0.5)
      cur_point.pose.position.z -= 0.1
      move_robot(cur_point.pose)
      rospy.sleep(0.5)
      grip("grip_on")
      rospy.sleep(0.5)
      cur_point.pose.position.z += 0.1
      move_robot(cur_point.pose)
      rospy.sleep(0.5)
      move_robot(fixed_rest)
      rospy.sleep(0.5)
      
      cur_point.pose.position.z += 0.015
      fixed.position.z += 0.1
      move_robot(fixed)
      rospy.sleep(0.5)
      fixed.position.z -= 0.1
      move_robot(fixed)
      rospy.sleep(0.5)
      grip("grip_off")
      fixed.position.z += 0.1
      move_robot(fixed)
      rospy.sleep(0.5)
   
   move_robot(fixed_rest)
   
 
if __name__ == "__main__":
   main()
