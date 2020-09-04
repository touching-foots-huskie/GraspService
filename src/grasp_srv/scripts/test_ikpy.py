import ikpy
import os
import numpy as np

file_path = os.path.dirname(os.path.abspath(__file__))
ur5e = ikpy.chain.Chain.from_urdf_file(file_path + "/../../ur_e_description/urdf/ur5e.urdf")
# ik_results = np.array([0., -1.24, -0.02, 0.55, 1.579, 0.034, -2.06, 0])
# ik_results = np.array([0., -1.24, 0., 0., 0., 0.0, 0., 0])
ik_results = np.array([0., 0., 0., 0., 0., 0.0, 0., 0])
reached_pose = ur5e.forward_kinematics(ik_results)
print(reached_pose)

for link in ur5e.links:
    if type(link) is not ikpy.link.OriginLink:
        print(link.rotation)