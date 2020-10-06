#!/usr/bin/python
import rospy
import numpy as np
import cv2  # OpenCV module
import time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from subt_msgs.msg import ArtifactPoseArray
from subt_msgs.msg import ArtifactPose

def talker():
	pub = rospy.Publisher('artifact_pose',  ArtifactPoseArray, queue_size = 1)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	n=0
	while not rospy.is_shutdown():
		obj_pose=ArtifactPose()
		obj_pose_arr=ArtifactPoseArray()
		obj_pose.pose.position.x=n
		obj_pose.pose.position.y=n+1
		obj_pose.pose.position.z=n+2
		n=n+1
		obj_pose.Class = "backpack"
		obj_pose_arr.pose_array=[]
		obj_pose_arr.count =1
		obj_pose_arr.pose_array.append(obj_pose)
		pub.publish(obj_pose_arr)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
