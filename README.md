# Grasp Pose Generation Service

## INPUT MSG
Object Pose is the input msg. See grasp_srv/msg/ObjectPoses.msg

Grasps is the output msg. See grasp_srv/msg/Grasps.msg

## Run Example

### Prerequisite
To run the service, you need to give a set of object_names and object_poses. Currently, the service relys on the "PCD" file.(More compacitiy will be added in the future version.) So you need to make sure there is a pcd file in "/root/ocrtoc_materials/models/${model_name}/meshes/cloud.pcd". 

To generate the corresponding PCD file, install the pcl_tools through the following command:

<pre><code>sudo apt-get install pcl_tools
</code></pre>

And then run the following command:

<pre><code>pcl_obj2pcd /root/ocrtoc_materials/models/${model_name}/meshes/textured.obj  /root/ocrtoc_materials/models/${model_name}/meshes/cloud.pcd
</code></pre>


### Run Service
<pre><code>cd GraspService
source devel/setup.bash
rosrun grasp_srv grasp_gen_server 
rosrun grasp_srv_grasp_gen_client #in another terminal
</code></pre>

## Visualization Test

To run visualization, just run the following code:
<pre><code>roslaunch grasp_srv grasp_gen.launch
</code></pre>

Everything is self-explained.

