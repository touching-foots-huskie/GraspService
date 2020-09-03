#!/usr/bin/env python
from __future__ import print_function

import ikpy
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def render_robot_pose(target_pose):
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

    redundant_times = 20  # 2 seconds 
    # load urdf robot
    import os
    file_path = os.path.dirname(os.path.abspath(__file__))
    ur5e = ikpy.chain.Chain.from_urdf_file(file_path + "/../../ur_e_description/urdf/ur5e.urdf")
    ik_results = ur5e.inverse_kinematics_frame(target_pose, orientation_mode="all")
    reached_pose = ur5e.forward_kinematics(ik_results)
    # create the msg publisher
    rospy.init_node('joint_state_publisher')
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)

    joint_msg = JointState()
    joint_msg.header = Header()
    for i, value in enumerate(ik_results):
        if(i == 0 or i == 7):
            continue
        else:
            joint_msg.name.append(ur5e.links[i].name)
            joint_msg.position.append(value)  
            
    for i, name in enumerate(empty_joints):
        joint_msg.name.append(name)
        joint_msg.position.append(0.)

    joint_msg.velocity = []
    joint_msg.effort = []

    # Kept sending
    for _ in range(redundant_times):
        joint_msg.header.stamp = rospy.Time.now()
        pub.publish(joint_msg)
        rate.sleep()


if __name__ == '__main__':
    target_pose = np.array([[1., 0., 0., 0.2],
                            [0., 1., 0., 0.2],
                            [0., 0., 1., 0.2],
                            [0., 0., 0., 1.]])
    # rospy.init_node('render_robot_pose', anonymous=True)
    render_robot_pose(target_pose)

    
