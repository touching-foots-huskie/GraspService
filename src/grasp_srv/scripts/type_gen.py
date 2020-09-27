import json

if __name__ == "__main__":
    name_file = "/root/GraspService/src/grasp_srv/grasp_data/object_id.json"
    type_file = "/root/GraspService/src/grasp_srv/grasp_data/grasp_mode.json"
    name_dict = dict()
    type_dict = dict()
    with open(name_file, "r") as f:
        name_dict = json.load(f)
    
    for key, value in name_dict.items():
        type_dict[value] = 1  # take box as default

    with open(type_file, "w") as f:
        json.dump(type_dict, f)