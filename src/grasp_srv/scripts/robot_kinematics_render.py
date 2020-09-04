#!/usr/bin/env python
from __future__ import print_function

import ikpy
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Pose

import waypoints_generate


def render_robot_pose(target_pose, object_name, object_pose):
    # Params for render
    empty_joints = [
        "robotiq_2f_85_right_driver_joint",
        "robotiq_2f_85_right_follower_joint",
        "robotiq_2f_85_right_pad_joint",
        "robotiq_2f_85_right_spring_link_joint",
        "robotiq_2f_85_left_driver_joint",
        "robotiq_2f_85_left_follower_joint",
        "robotiq_2f_85_left_pad_joint",
        "robotiq_2f_85_left_spring_link_joint"]
    active_links_mask = [False, True, True, True,
                        True,  True, True, False]

    redundant_times = 20  # 2 seconds 
    # load urdf robot
    import os
    file_path = os.path.dirname(os.path.abspath(__file__))
    ur5e = ikpy.chain.Chain.from_urdf_file(file_path + "/../../ur_e_description/urdf/ur5e.urdf",
                                           active_links_mask=active_links_mask)

    print(ur5e.links[-1].name)
    # generadually change into target
    waypoints_num = 5
    j_value = np.zeros([8,])

    iter_num = 0
    diff_pose_sum = 1.

    while iter_num < 10 and diff_pose_sum > 0.1:
        m_start = ur5e.forward_kinematics(j_value)
        waypoints = waypoints_generate.m_waypoints(m_start, target_pose, waypoints_num)
        for i, waypoint in enumerate(waypoints):
            j_value = ikpy.inverse_kinematics.inverse_kinematic_optimization(
                        ur5e, waypoint, j_value, 
                        max_iter = 100, 
                        orientation_mode=None, no_position=False)
            j_value = ikpy.inverse_kinematics.inverse_kinematic_optimization(
                        ur5e, waypoint, j_value, 
                        max_iter = 100, 
                        orientation_mode="all", no_position=False)

        reached_pose = ur5e.forward_kinematics(j_value)
        diff_pose = target_pose - reached_pose
        diff_pose_sum = np.linalg.norm(diff_pose)
        iter_num += 1

    # create the msg publisher
    rospy.init_node('joint_state_publisher')
    
    # publisher
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    obj_pub = rospy.Publisher("object_name", String, queue_size=10) 
    pose_pub = rospy.Publisher("object_pose", Pose, queue_size=10)
    rate = rospy.Rate(10)

    joint_msg = JointState()
    joint_msg.header = Header()
    for i, value in enumerate(j_value):
        if i == 0 or i == 7:
            continue
        else:
            joint_msg.name.append(ur5e.links[i].name)
            joint_msg.position.append(value)  
            
    for i, name in enumerate(empty_joints):
        joint_msg.name.append(name)
        joint_msg.position.append(0.)

    joint_msg.velocity = []
    joint_msg.effort = []

    object_name_msg = String()
    object_name_msg.data = object_name
    # Publishing sending
    for _ in range(redundant_times):
        joint_msg.header.stamp = rospy.Time.now()
        pub.publish(joint_msg)
        obj_pub.publish(object_name_msg)
        pose_pub.publish(object_pose)
        rate.sleep()


if __name__ == '__main__':
    pass

    
