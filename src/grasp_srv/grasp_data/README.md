# Grasp Mode

## Grasp Mode Code
- box : 1
- can : 2
- bowl : 3
- horizontal_block : (4, 8)
- vertical_block : (16, 32)
- pre-defined : 64
- gpd : 128
- flat : 256

## Grasp Mode Explanation
- box : The grasp service will generate pre-defined pose assuming this is a cube-like item.
- can : The grasp service will generate pre-defined pose assuming this is a cylinder-like item.
- pre-defined : The grasp service will reload the pre-defined poses from "ocrtoc_grasping/grasp_srv/grasp_data/{MODEL NAME}/pose.json".
- gpd : The grasp service will call gpd to generate grasp pose based on the pcd model.
- bottle : The grasp service will only generate horizontal grasp pose for bottle like object. 
- plate : The grasp service will only generate suitable grasp for plate-like object.
- bowl : The grasp service will generate grasp pose which can grasp bowl from edges.
- flat : This object is flat and hard to grasp.

## Grasp Mode Code Combination
Grasp mode code support combination. For example, if you want to generate a grasp pose both from box mode and gpd mode, you can edit the corresponding code equals to 1+8=9.

The outputing grasp poses will be combined with the following order "box"-"can"-"pre-defined"-"gpd"-"bottle"-"plate"-"bowl"-"flat".
