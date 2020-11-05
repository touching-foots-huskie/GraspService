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
image1 = np.zeros([480, 640]);
image2 = np.zeros([480, 640]);
image3 = np.zeros([480, 640]);

model_name = ""
grasp_id = 0
data_dir = os.path.join(os.path.dirname(__file__), '../grasp_data/')
# data_dir = "/root/GraspService/src/grasp_srv/grasp_data/"
model_name_lock = False
grasp_id_lock = False
image_1_lock = False
image_2_lock = False
image_3_lock = False


def pose_callback(grasp_pose):
    global model_name_lock
    global grasp_id_lock
    global image_1_lock
    global image_2_lock
    global image_3_lock
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

    while(True):
        if model_name_lock and grasp_id_lock:
            # save image
            rospy.sleep(1.0)  # wait for rviz to response
            image_1_lock = image_2_lock = image_3_lock = True
            
            image_dir = "{}{}".format(data_dir, model_name)
            if not os.path.isdir(image_dir):
                os.mkdir(image_dir)

            # save upper
            image1_name = "{}/{}_upper.jpg".format(image_dir, grasp_id)
            cv2.imwrite(image1_name, image1)
            # save front
            image2_name = "{}/{}_front.jpg".format(image_dir, grasp_id)
            cv2.imwrite(image2_name, image2)
            # save right
            image3_name = "{}/{}_right.jpg".format(image_dir, grasp_id)
            cv2.imwrite(image3_name, image3)

            # release lock
            model_name_lock = False
            grasp_id_lock = False
            image_1_lock = image_2_lock = image_3_lock = False
            break
        # sleep once
        rospy.sleep(0.1)

    return


def camera1_callback(image_msg):
    global image1
    if not image_1_lock:
        bridge = CvBridge()
        image1 = bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        return


def camera2_callback(image_msg):
    global image2
    if not image_2_lock:
        bridge = CvBridge()
        image2 = bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        return


def camera3_callback(image_msg):
    global image3
    if not image_3_lock:
        bridge = CvBridge()
        image3 = bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        return


def model_name_callback(model_name_msg):
    global model_name
    global model_name_lock
    if not model_name_lock:
        model_name = model_name_msg.data
        model_name_lock = True
        return


def grasp_id_callback(grasp_id_msg):
    global grasp_id
    global grasp_id_lock
    if not grasp_id_lock:
        grasp_id = grasp_id_msg.data
        grasp_id_lock = True
        return


if __name__ == "__main__":
    rospy.init_node('grasp_connector')
    # start grasp subscriber
    rospy.Subscriber("grasp_pose", geometry_msgs.msg.Pose, pose_callback)

    # image subscriber
    rospy.Subscriber("rviz1/camera1/image", sensor_msgs.msg.Image, camera1_callback)
    rospy.Subscriber("rviz1/camera2/image", sensor_msgs.msg.Image, camera2_callback)
    rospy.Subscriber("rviz1/camera3/image", sensor_msgs.msg.Image, camera3_callback)

    # model_name subscriber
    rospy.Subscriber("object_name", std_msgs.msg.String, model_name_callback)

    # grasp_id subscriber
    rospy.Subscriber("grasp_id", std_msgs.msg.Int32, grasp_id_callback)

    # grasp_id subscriber
    rospy.spin()