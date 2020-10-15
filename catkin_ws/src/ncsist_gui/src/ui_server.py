#!/usr/bin/env python

import numpy as np
import cv2
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
import math
import tf

from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from geometry_msgs.msg import PoseArray, Pose,PoseStamped,Twist

import message_filters

from rospy_tutorials.srv import *
from subt_msgs.srv import int8
from subt_msgs.msg import ArtifactPoseArray
from subt_msgs.msg import ArtifactPose
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int64MultiArray

class GUI_server():
	def __init__(self):
		rospy.loginfo("Wait for Camera Info")
		info = rospy.wait_for_message('camera/color/camera_info',CameraInfo)
		self.fx = info.P[0]
		self.fy = info.P[5]
		self.cx = info.P[2]
		self.cy = info.P[6]
		print "camera_info: ", self.fx, self.fy, self.cx, self.cy

		## for opencv
		self.cv_bridge = CvBridge()

		## tf listener
		self.listener = tf.TransformListener()

		## msg filter sync rgb and d images
		image_sub1 = message_filters.Subscriber('camera/color/image_raw', Image)
		depth_sub1 = message_filters.Subscriber('camera/aligned_depth_to_color/image_raw', Image)
		ts1 = message_filters.ApproximateTimeSynchronizer([image_sub1, depth_sub1], 10, slop=0.1)
		ts1.registerCallback(self.img_cb)

		# Publisher & Subscriber
		self.pub_artifact_pose = rospy.Publisher("artifact_pose", ArtifactPoseArray, queue_size = 1)
		rospy.Subscriber("ui_position", Int16MultiArray, self.ui_position_callback)
		rospy.Subscriber("/box_pred/box_coords", Int64MultiArray, self.bbox_callback)
		rospy.Subscriber('deal_with_goBack', String, self.cb_deal_with_goBack, queue_size=1)


		self.movegoal=PoseStamped()
		self.obj_pose=ArtifactPose()
		self.obj_pose_arr=ArtifactPoseArray()
		self.obj_pose.probability=1
		self.obj_pose.Class="backpack"
		self.obj_pose_arr.header.frame_id="camera_middle_link"
		self.start_point=ArtifactPose()


		self.gui_x= -1
		self.gui_y= -1



		rospy.loginfo("ui_server init done")




	def cb_deal_with_goBack(self, msg):
		if msg.data=="auto_move_back":
			rospy.loginfo("auto_move_back!!!!!!")
			self.obj_pose.pose = self.start_point.pose
			self.obj_pose_arr.pose_array=[]
			self.obj_pose_arr.pose_array.append(self.obj_pose)
			self.pub_artifact_pose.publish(self.obj_pose_arr)

		if msg.data=="record position":
			rospy.loginfo("record position")
			dum_posestamp = PoseStamped()
			dum_posestamp.pose.position.x = 0
			dum_posestamp.pose.position.y = 0
			dum_posestamp.pose.position.z = 0
			dum_posestamp.header.frame_id = "base_link"
			try:
				dum_posestamp = self.listener.transformPose("map", dum_posestamp)
			except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				rospy.loginfo("faile to catch tf %s 2 %s" %("map", "base_link"))
				return
			self.start_point.pose = dum_posestamp.pose
			print('self.start_point:',self.start_point)


	def ui_position_callback(self,msg):
		print(msg)
		# rospy.loginfo("ui_position_callback")
		self.gui_x= msg.data[0]
		self.gui_y= msg.data[1]
		rospy.logwarn("I heard gui_x=%s  gui_y=%s", msg.data[0],msg.data[1])
		self.pub_artifact_pose.publish(self.obj_pose_arr)
		rospy.logwarn("pub_artifact_pose")
		return

	def bbox_callback(self,msg):
		# print(msg.data)
		if len(msg.data)==4:
			# rospy.loginfo("bbox_callback")
			self.gui_x= (msg.data[0]+msg.data[2])/2
			self.gui_y= (msg.data[1]+msg.data[3])/2
			rospy.logwarn("I heard bbox_center_x=%s  bbox_center_y=%s", self.gui_x,self.gui_y)
			self.pub_artifact_pose.publish(self.obj_pose_arr)
			rospy.logwarn("pub_artifact_pose")
		return	

	def img_cb(self, rgb_msg, depth_msg):
		rospy.loginfo("img_cb")

		cv_image = self.cv_bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
		cv_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, "16UC1")

		zc = cv_depth[self.gui_y, self.gui_x]
		zc = float(zc)/1000. # 1000. for D435
		rx, ry, rz = self.getXYZ(self.gui_x /1.0 , self.gui_y, zc/1.0 )

		dum_posestamp = PoseStamped()
		dum_posestamp.pose.position.x = rx
		dum_posestamp.pose.position.y = ry
		dum_posestamp.pose.position.z = rz
		dum_posestamp.header = rgb_msg.header
		dum_posestamp.header.frame_id = "camera_middle_link"
		try:
			dum_posestamp = self.listener.transformPose("map", dum_posestamp)
		except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.loginfo("faile to catch tf %s 2 %s" %
				("map", rgb_msg.header.frame_id))
			return

		self.obj_pose.pose = dum_posestamp.pose

		self.obj_pose_arr.header.stamp=rgb_msg.header.stamp
		self.obj_pose_arr.count =1
		self.obj_pose_arr.camera = "camera"

		self.movegoal.header=self.obj_pose_arr.header
		self.movegoal.pose=self.obj_pose
		self.obj_pose_arr.pose_array=[]
		self.obj_pose_arr.pose_array.append(self.obj_pose)




	def getXYZ(self, x, y, zc):
		x = float(x)
		y = float(y)
		zc = float(zc)
		inv_fx = 1.0/self.fx
		inv_fy = 1.0/self.fy
		x = (x - self.cx) * zc * inv_fx
		y = (y - self.cy) * zc * inv_fy
		return zc, -1*x, -1*y



if __name__ == '__main__':
	rospy.init_node('ui_server_node',anonymous=False)
	duckie_detection = GUI_server()
	rospy.spin()
