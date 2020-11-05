import os
import json

if __name__ == "__main__":
    data_dir = os.path.join(os.path.dirname(__file__), '../grasp_data/')
    name_file = data_dir + "object_id.json"
    type_file = data_dir + "grasp_mode.json"
    name_dict = dict()
    type_dict = dict()
    with open(name_file, "r") as f:
        name_dict = json.load(f)
    
    for key, name in name_dict.items():
        grasp_mode = dict()
        if "box" in name or "cube" in name or "block" in name:
            grasp_mode["basic_type"] = ["box"];
            grasp_mode["block_list"] = [False, False, False, False, False, False, False, False, False, False, False, False];
            grasp_mode["hard"] = False
        elif "cup" in name or "can" in name or "cocacola" in name or "pepsi" in name or "redbull" in name or "arrow" in name:
            grasp_mode["basic_type"] = ["can"];
            grasp_mode["block_list"] = [False, False, False, False, False, False, False, False, False, False, False, False];
            grasp_mode["hard"] = False
        elif "plate" in name:
            grasp_mode["basic_type"] = ["can"];
            grasp_mode["block_list"] = [True, False, True, True, False, False, False, False, False, False, False, False];
            grasp_mode["hard"] = False
        elif "bottle" in name:
            grasp_mode["basic_type"] = ["box"];
            grasp_mode["block_list"] = [False, False, False, False, False, False, False, False, True, True, True, True];
            grasp_mode["hard"] = False
        elif "bowl" in name:
            grasp_mode["basic_type"] = ["bowl"];
            grasp_mode["block_list"] = [False, False, False, False, False, False, False, False, False, False, False, False];
            grasp_mode["hard"] = False
        else:
            # default as box for now
            grasp_mode["basic_type"] = ["box"];
            grasp_mode["block_list"] = [False, False, False, False, False, False, False, False, False, False, False, False];
            grasp_mode["hard"] = False
        # special structure
        if name in ["wooden_boat_bowl", "wooden_square_bowl"]:
            grasp_mode["basic_type"] = ["square_bowl"];
            grasp_mode["block_list"] = [False, False, False, False, False, False, False, False, False, False, False, False];
            grasp_mode["hard"] = False
        elif name in ["wooden_cup1", "wooden_cup2", "wooden_cup3"]:
            grasp_mode["basic_type"] = ["box"];
            grasp_mode["block_list"] = [False, False, False, False, False, False, False, False, False, False, False, False];
            grasp_mode["hard"] = False
            
        type_dict[name] = grasp_mode

    
    with open(type_file, "w") as f:
        json.dump(type_dict, f)