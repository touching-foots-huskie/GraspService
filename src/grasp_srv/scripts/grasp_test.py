#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import grasp_srv.srv
import grasp_srv.msg
import geometry_msgs.msg
import sensor_msgs.msg
import grasp_srv.msg

import numpy as np
from scipy.spatial.transform import Rotation as R

import gripper_visualization


'''
Pose is a 1D list of 7 elements (x, y, z, qx, qy, qz, qw), 
'''
def grasp_callback(object_poses):
    rospy.wait_for_service('grasp_gen')
    try:
        grasp_gen = rospy.ServiceProxy('grasp_gen', grasp_srv.srv.GraspGen)
        resp = grasp_gen(object_poses)
        # parse msg
        if len(resp.grasps.global_grasp_poses) > 0:
            if len(resp.grasps.global_grasp_poses[0].grasp_poses) > 0:
                # Test on One
                grasp_pose   = resp.grasps.global_grasp_poses[0].grasp_poses[0]
                object_name  = object_poses.object_names[0]
                object_scale = object_poses.object_scales[0]
                object_pose  = object_poses.object_poses[0]

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
                gripper_visualization.publish_object(object_pose, object_name, object_scale)
                gripper_visualization.publish_gripper(
                    r_quat, 
                    [grasp_pose.position.x,
                    grasp_pose.position.y,
                    grasp_pose.position.z],
                    0)
            
                rospy.loginfo("Pose Rendered")
                return
            else:
                rospy.loginfo("No Grasp Candidates Found | CODE:2")
        else:
            rospy.loginfo("No Grasp Candidates Found | CODE:1")
                
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s"%e)


if __name__ == "__main__":
    rospy.init_node('grasp_connector')
    # start grasp subscriber
    rospy.Subscriber("object_poses", grasp_srv.msg.ObjectPoses, grasp_callback)
    rospy.spin()

