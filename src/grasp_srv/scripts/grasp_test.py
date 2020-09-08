#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import grasp_srv.srv
import grasp_srv.msg
import geometry_msgs.msg
import sensor_msgs.msg

import numpy as np
import ikpy
from scipy.spatial.transform import Rotation as R

import gripper_visualization


'''
Pose is a 1D list of 7 elements (x, y, z, qx, qy, qz, qw), 
'''
def test_grasp_gen(model_name_list, object_pose_list, object_scale_list):
    rospy.init_node('grasp_state_publisher')
    pub = rospy.Publisher('joint_states', sensor_msgs.msg.JointState, queue_size=10)
    
    rospy.wait_for_service('grasp_gen')
    try:
        grasp_gen = rospy.ServiceProxy('grasp_gen', grasp_srv.srv.GraspGen)
        object_poses = grasp_srv.msg.ObjectPoses()
        # create msg
        for name, pose, scale in zip(model_name_list, object_pose_list, object_scale_list):
            object_poses.object_names.append(name)
            object_poses.object_scales.append(scale)
            object_pose = geometry_msgs.msg.Pose()
            # set position
            object_pose.position.x = pose[0]
            object_pose.position.y = pose[1]
            object_pose.position.z = pose[2]
            object_pose.orientation.x = pose[3]
            object_pose.orientation.y = pose[4]
            object_pose.orientation.z = pose[5]
            object_pose.orientation.w = pose[6]
            object_poses.object_poses.append(object_pose)

        resp = grasp_gen(object_poses)
        # parse msg
        if len(resp.grasps.global_grasp_poses[0].grasp_poses) > 0:
            print("Service Sent")
            # Test on One
            grasp_pose = resp.grasps.global_grasp_poses[0].grasp_poses[0]
            object_name = model_name_list[0]
            object_pose = object_poses.object_poses[0]

            # publish object & gripper
            gripper_visualization.publish_object(object_pose, object_name)
            gripper_visualization.publish_gripper(
                object_pose.orientation, 
                object_pose.position,
                0)
        
            print("Pose Rendered")
            return
        else:
            print("No Grasp Candidates Found")

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    model_name_list = ["master_chef_can"]
    origin_pos = [0.4, 0.4, 0.2, 0., 0., 0., 1.]
    object_pose_list = [origin_pos]
    object_scale_list = [0.8]
    test_grasp_gen(model_name_list, object_pose_list, object_scale_list)

