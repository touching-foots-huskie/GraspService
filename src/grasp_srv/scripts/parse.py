import xml.etree.ElementTree as ET
import json
import os

if __name__ == "__main__":
    scene_dir = "/root/ocrtoc_materials/scenes/"
    output_dir = "/root/GraspService/src/grasp_srv/kinect_data/output/"
    for scene_name in os.listdir(scene_dir):
        file_name = scene_dir + scene_name + "/input.world"

        model_scales = dict()
        tree = ET.parse(file_name)
        root = tree.getroot()
        for child in root[0]:
            if child.tag == "model":
                scales = child[0][0][0][0][1].text
                model_name = child[0][0][0][0][0].text.split("/")[2]
                model_scales[model_name] = float(scales.split(" ")[0])

        output_path = output_dir + scene_name + "/scale.json"
        with open(output_path, "w") as f:
            json.dump(model_scales, f)
