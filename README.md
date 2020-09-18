# Grasp Pose Generation Service

## INPUT MSG
Object Pose is the input msg. See grasp_srv/msg/ObjectPoses.msg

Grasps is the output msg. See grasp_srv/msg/Grasps.msg

To Save grasp pose, see grasp_srv/msg/SaveGrasp.msg
## Run Example

### Prerequisite
Install this repo:
<pre><code>https://github.com/nlohmann/json.git
</code></pre>


### Run Service
<pre><code>roslaunch grasp_srv grasp_gen.launch
</code></pre>

You can change DATA_PATH (Scene Info Path), MODEL_PATH (PointCloud Model), GRASP_PATH (Grasp Pose DataBase).
