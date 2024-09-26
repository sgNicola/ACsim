import json
import pandas as pd
import os
import motmetrics as mm
from scipy.spatial.distance import cdist
from perceptionIdentify.intervention.utilities import ObjectUtils
import perceptionIdentify.tools.transforms as trans
from perceptionIdentify.tools.msg_serialize import MessageSerialize
import numpy as np
import re
from param import ObjectData_dir, Data_dir, failure_mode
from perceptionIdentify.parse_fusion import get_failures, get_groundtruth,create_experiment_file,get_directory,read_json_files,object_to_dataframe,evaluate_detection

node_topics = {'lidar_centerpoint': '/perception/object_recognition/detection/centerpoint/objects', 
               'clustering_shape_estimation': '/perception/object_recognition/detection/clustering/objects', 
               'euclidean_cluster': '/perception/object_recognition/detection/clustering/clusters', 
               'object_association_merger_1': '/perception/object_recognition/detection/objects', 
               'obstacle_pointcloud_based_validator': '/perception/object_recognition/detection/centerpoint/validation/objects',
                'multi_object_tracker': '/perception/object_recognition/tracking/objects',
                'groundtruth': '/carla_actors'}

detection_nodes = ['lidar_centerpoint',
                   'clustering_shape_estimation',
                   'euclidean_cluster',
                   'object_association_merger_1',
                   'obstacle_pointcloud_based_validator'
]

def get_faults(node_label,rosbagname,groundtruth):
    publish_topic = node_topics.get(node_label)
    actor_directory = get_directory(rosbagname,publish_topic)
    actor_data =  read_json_files(actor_directory,node_label)
    detected_dataframe =  object_to_dataframe(actor_data)
    evaluation_results = evaluate_detection(groundtruth, detected_dataframe)
    return evaluation_results

def save_lidar_failure(target_objects,groundtruth_detection,rosbagname,bagname,
                   detection_nodes,evaluation_failures, exp_id):
    pattern = r'object_(\d+)_{}'.format(failure_mode)
    default_fault_modes = ['wrong_classification', 'false_negative', 'wrong_localization',"false_positive"]
    f_pattern = re.compile(pattern)
    # Extracting object IDs that have the false_negative fault mode
    object_ids = set(f_pattern.search(index).group(1) 
                            for index in target_objects if f_pattern.search(index))
        # Check if object_ids_with_fn is not null (empty)
    object_ids = sorted(map(int, object_ids))
    print("ids",object_ids)
    if not object_ids:
        # Handle the case when object_ids_with_fn is empty
        print(f"No objects found with failure mode: {failure_mode}")
        return
    for object_id in object_ids:
        failure= evaluation_failures[f'object_{object_id}_{failure_mode}']
        data = pd.DataFrame()
        data[f'multi_object_tracker_{failure_mode}'] = failure
        for node_label in detection_nodes:
            if node_label in ['clustering_shape_estimation','euclidean_cluster']:
                fault_modes =['false_negative', 'wrong_localization',"false_positive"]
            else:
                fault_modes =default_fault_modes
            faults,fault_objects = get_faults(node_label, rosbagname,groundtruth_detection)
            for mode in fault_modes:
                fault_mode_object = f'object_{object_id}_{mode}'
                if fault_mode_object in fault_objects:
                    data[f'{node_label}_{mode}'] = faults[fault_mode_object]
        for mode in fault_modes:
                fault_mode_object = f'object_{object_id}_{mode}'
                    # Define the file path
        file_name = f'{bagname}_object_{object_id}_{failure_mode}.csv'
        exp_path = create_experiment_file(exp_id)
        file_path = os.path.join(exp_path, file_name)
        data.to_csv(file_path, index=False)
        return object_id
 
def search_fault_targets(object_id, evaluation_failures,target_objects,rosbagname,
                    groundtruth_detection,exp_id,bagname):
    target_failure =  f'object_{object_id}_{failure_mode}'
    data = pd.DataFrame()
    if target_failure not in target_objects:
        print(f"No data found for object ID {object_id} with failure mode: {failure_mode}")
    else:
        failure= evaluation_failures[target_failure]
        data[f'multi_object_tracker_{failure_mode}'] = failure   
    default_fault_modes = ['wrong_classification', 'false_negative', 'wrong_localization',"false_positive"]
    for node_label in detection_nodes:
        if node_label in ['clustering_shape_estimation','euclidean_cluster']:
            fault_modes =['false_negative', 'wrong_localization',"false_positive"]
        else:
            fault_modes =default_fault_modes
        faults,fault_objects = get_faults(node_label, rosbagname,groundtruth_detection)
        for mode in fault_modes:
            fault_mode_object = f'object_{object_id}_{mode}'
            if fault_mode_object in fault_objects:
                data[f'{node_label}_{mode}'] = faults[fault_mode_object]
    for mode in fault_modes:
            fault_mode_object = f'object_{object_id}_{mode}'
    file_name = f'{bagname}_object_{object_id}_{failure_mode}.csv'
    exp_path = create_experiment_file(exp_id)
    file_path = os.path.join(exp_path, file_name)
    data.to_csv(file_path, index=False)      

def record_failure(bagname,exp_id):
    rosbagname = os.path.join(exp_id,bagname)
    groundtruth_tracking, groundtruth_detection = get_groundtruth(rosbagname)
    evaluation_failures, failure_objects = get_failures(rosbagname,groundtruth_tracking)
    num_rows = len(evaluation_failures)
    target_objects =[]
    # only consider the fauilure that more than half
    for object in failure_objects:
        num_ones = evaluation_failures[object].sum()
        if num_ones >= (num_rows / 2):
            target_objects.append(object)
    record_id= save_lidar_failure(target_objects,groundtruth_detection,rosbagname,bagname,
                       detection_nodes,evaluation_failures,exp_id)
    return record_id

def record_intervene(object_id, bagname,exp_id):
    rosbagname = os.path.join(exp_id,bagname)
    groundtruth_tracking, groundtruth_detection = get_groundtruth(rosbagname)
    evaluation_failures, failure_objects = get_failures(rosbagname,groundtruth_tracking)
    num_rows = len(evaluation_failures)
    target_objects =[]
    for object in failure_objects:
        num_ones = evaluation_failures[object].sum()
        if num_ones >= (num_rows / 2):
            target_objects.append(object)   
    search_fault_targets(object_id, evaluation_failures,target_objects,rosbagname, 
                   groundtruth_detection,exp_id,bagname)
    
if __name__ == "__main__":
    # failure_mode = "false_negative"
    bagname = "03"
    exp_id = "1"
    object_id = "21"
    if  object_id is None:
        object_id=record_failure(bagname,exp_id)
        print(object_id)
    else:
        record_intervene(object_id, bagname,exp_id)