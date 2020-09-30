# Grasp Mode

## Basic Type
- box
- can
- bowl
- pre-defined
- gpd

## Grasp Mode Explanation
- box : The grasp service will generate pre-defined pose assuming this is a cube-like item.
- can : The grasp service will generate pre-defined pose assuming this is a cylinder-like item.
- pre-defined : The grasp service will reload the pre-defined poses from "ocrtoc_grasping/grasp_srv/grasp_data/{MODEL NAME}/pose.json".
- gpd : The grasp service will call gpd to generate grasp pose based on the pcd model.
- bowl : The grasp service will generate grasp pose which can grasp bowl from edges.

## Block List
For box shape, block-list can block the grasp pose from directions [+x, -x, +y, -y, +z, -z].

For can shape, block-list can block the grasp pose from directions [+h, -h, +v, -v].