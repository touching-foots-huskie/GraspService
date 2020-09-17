#!/usr/bin/env python
from __future__ import print_function

import rospy
import geometry_msgs.msg

import numpy as np
from scipy.spatial.transform import Rotation as R

import gripper_visualization


'''
Pose is a 1D list of 7 elements (x, y, z, qx, qy, qz, qw), 
'''
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


if __name__ == "__main__":
    rospy.init_node('grasp_connector')
    # start grasp subscriber
    rospy.Subscriber("grasp_pose", geometry_msgs.msg.Pose, pose_callback)
    rospy.spin()