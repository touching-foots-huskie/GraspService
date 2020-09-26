#!/usr/bin/env python
from __future__ import print_function

# ros
import rospy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

import numpy as np
from scipy.spatial.transform import Rotation as R

import gripper_visualization

# cv
from cv_bridge import CvBridge
import numpy as np
import cv2

# os
import os


# global data
image1 = np.zeros([640, 480]);
image2 = np.zeros([640, 480]);
image3 = np.zeros([640, 480]);

model_name = ""
grasp_id = 0
data_dir = "/root/GraspService/grasp_data/"


def pose_callback(grasp_pose):
    # publish object & gripper
    # Rot from ee to tool0
    r = R.from_quat(np.array([grasp_pose.orientation.x,
                              grasp_pose.orientation.y,
                              grasp_pose.orientation.z,
                              grasp_pose.orientation.w]))
    rz = R.from_rotvec(-np.pi/2.0*np.array([0., 0., 1.]))
    rx = R.from_rotvec(-np.pi/2.0*np.array([1., 0., 0.]))
    rf = r * rz * rx
    r_quat = rf.as_quat()
    gripper_visualization.publish_gripper(
        r_quat, 
        [grasp_pose.position.x,
         grasp_pose.position.y,
         grasp_pose.position.z],
         0)
    rospy.loginfo("Pose Rendered")
    return


def camera1_callback(image_msg):
    bridge = CvBridge()
    image1 = bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
    return


def model_name_callback(model_name_msg):
    model_name = model_name_msg.data
    return


def grasp_id_callback(grasp_id_msg):
    grasp_id = grasp_id_msg.data
    # save image
    image_dir = "{}{}".format(data_dir, model_name)
    if not os.path.isdir(image_dir):
        os.mkdir(image_dir)

    image1_name = "{}/{}_upper.img".format(image_dir, grasp_id)
    cv2.imwrite(image1_name, image1)
    return


if __name__ == "__main__":
    rospy.init_node('grasp_connector')
    # start grasp subscriber
    rospy.Subscriber("grasp_pose", geometry_msgs.msg.Pose, pose_callback)

    # image subscriber
    rospy.Subscriber("rviz1/camera1/image", sensor_msgs.msg.Image, camera1_callback)

    # model_name subscriber
    rospy.Subscriber("object_name", std_msgs.msg.String, model_name_callback)

    # grasp_id subscriber
    rospy.Subscriber("grasp_id", std_msgs.msg.Int32, grasp_id_callback)

    # grasp_id subscriber
    rospy.spin()