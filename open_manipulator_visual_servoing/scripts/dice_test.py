#!/usr/bin/env python


import rospy
from open_manipulator_visual_servoing.srv import *



def indexer_client(u, v):
    rospy.wait_for_service('visual_servoing/depth_cloud/indexer')
    try:
        indexer = rospy.ServiceProxy('visual_servoing/depth_cloud/indexer', IndexPointCloud)
        resp = indexer(u, v)
        return resp.point.x, resp.point.y, resp.point.z
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def blob_client(colors):
    rospy.wait_for_service('visual_servoing/color/blob_finder')
    try:
        blob_finder = rospy.ServiceProxy('visual_servoing/color/blob_finder', LocateBlob)
        resp = blob_finder(colors)
        return resp.x, resp.y
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
        
        
red_range = [0, 0, 80, 60, 30, 210]
red_uv = blob_client(red_range)
red_xyz = indexer_client(red_uv[0], red_uv[1])

print "RED:"
print "\tRange: " + str(red_range)
print "\tUV: " + str(red_uv)
print "\tXYZ: " + str(red_xyz)



green_range = [40, 80, 0, 120, 255, 50]
green_uv = blob_client(green_range)
green_xyz = indexer_client(green_uv[0], green_uv[1])

print "GREEN:"
print "\tRange: " + str(green_range)
print "\tUV: " + str(green_uv)
print "\tXYZ: " + str(green_xyz)



blue_range = [150, 50, 0, 255, 150, 50]
blue_uv = blob_client(blue_range)
blue_xyz = indexer_client(blue_uv[0], blue_uv[1])

print "BLUE:"
print "\tRange: " + str(blue_range)
print "\tUV: " + str(blue_uv)
print "\tXYZ: " + str(blue_xyz)
