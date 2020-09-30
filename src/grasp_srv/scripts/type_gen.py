import json

if __name__ == "__main__":
    name_file = "/root/GraspService/src/grasp_srv/grasp_data/object_id.json"
    type_file = "/root/GraspService/src/grasp_srv/grasp_data/grasp_mode.json"
    name_dict = dict()
    type_dict = dict()
    with open(name_file, "r") as f:
        name_dict = json.load(f)
    
    for key, name in name_dict.items():
        grasp_mode = dict()
        if "box" in name or "cube" in name or "block" in name:
            grasp_mode["basic_types"] = ["box"];
            grasp_mode["block_list"] = [False, False, False, False, False, False, False, False, False, False, False, False];
            grasp_mode["hard"] = False
        elif "cup" in name or "can" in name:
            grasp_mode["basic_types"] = ["can"];
            grasp_mode["block_list"] = [False, False, False, False, False, False, False, False, False, False, False, False];
            grasp_mode["hard"] = False
        elif "plate" in name:
            grasp_mode["basic_types"] = ["can"];
            grasp_mode["block_list"] = [True, False, True, True, False, False, False, False, False, False, False, False];
            grasp_mode["hard"] = False
        elif "bottle" in name:
            grasp_mode["basic_types"] = ["box"];
            grasp_mode["block_list"] = [False, False, False, False, False, False, False, False, True, True, True, True];
            grasp_mode["hard"] = False
        else:
            # default as box for now
            grasp_mode["basic_types"] = ["box"];
            grasp_mode["block_list"] = [False, False, False, False, False, False, False, False, False, False, False, False];
            grasp_mode["hard"] = False
        type_dict[name] = grasp_mode

    with open(type_file, "w") as f:
        json.dump(type_dict, f)