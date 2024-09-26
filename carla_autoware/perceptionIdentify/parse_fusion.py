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
node_topics = {'lidar_centerpoint': '/perception/object_recognition/detection/centerpoint/objects', 
               'detected_object_feature_remover': '/perception/object_recognition/detection/clustering/camera_lidar_fusion/objects', 
               'roi_cluster_fusion': '/perception/object_recognition/detection/clustering/camera_lidar_fusion/clusters', 
               'clustering_shape_estimation': '/perception/object_recognition/detection/clustering/objects_with_feature', 
               'fusion_shape_estimation': '/perception/object_recognition/detection/clustering/camera_lidar_fusion/objects_with_feature', 
               'euclidean_cluster': '/perception/object_recognition/detection/clustering/clusters', 
               'object_association_merger_0': '/perception/object_recognition/detection/camera_lidar_fusion/objects', 
               'object_association_merger_1': '/perception/object_recognition/detection/objects', 
               'obstacle_pointcloud_based_validator': '/perception/object_recognition/detection/centerpoint/validation/objects',
                'tensorrt_yolo': '/perception/object_recognition/detection/rois0', 
                'multi_object_tracker': '/perception/object_recognition/tracking/objects',
                'groundtruth': '/carla_actors'}

detection_nodes = ['lidar_centerpoint',
                   'detected_object_feature_remover',
                   'roi_cluster_fusion',
                   'fusion_shape_estimation',
                   'clustering_shape_estimation',
                   'euclidean_cluster',
                   'object_association_merger_0',
                   'object_association_merger_1',
                   'obstacle_pointcloud_based_validator'
]

def tf_pose_relative_ego(frame):
    # trans the groundtruth position in carla to autoware frame
    for obj in frame:
        if obj.get('type') == 'ego_vehicle':
            ego_vehicle_object = obj
    obstacles = [obj for obj in frame if obj.get('type') != 'ego_vehicle']
    obstacle_transform_list =[]
    for obstacle in obstacles:
        transformed_obs = obstacle.copy() 
        obstacle_transform_matrix = trans.ros_pose_to_transform_matrix(
            MessageSerialize.dict2pose(obstacle["pose"]))
        ego_transform_matrix = trans.ros_pose_to_transform_matrix(
            MessageSerialize.dict2pose(ego_vehicle_object["pose"]))
        relative_transform_matrix = np.matrix(
        ego_transform_matrix).getI() * np.matrix(obstacle_transform_matrix)
        obstacle_transform = MessageSerialize.pose_serialize(
            trans.transform_matrix_to_ros_pose(relative_transform_matrix)
        )
        transformed_obs['pose'] = obstacle_transform
        obstacle_transform_list.append(transformed_obs)
    return  obstacle_transform_list

def get_directory(rosbagname,topic):
    RosbagName_dir =  ObjectData_dir+rosbagname
    topic_folder = topic.replace("/","_")
    base_directory  = os.path.join(RosbagName_dir,topic_folder)
    return base_directory

def read_json_files(directory,node_label):
    json_data = []
    file_list = os.listdir(directory)
    file_list = sorted(file_list, key=lambda x: int(x.split(".")[0]), reverse=False)
    for filename in file_list:
        if filename.endswith(".json"):
            timestamp = int(filename.split(".")[0])  # get timestamp
            filepath = os.path.join(directory, filename)
            try:
                with open(filepath, "r") as file:
                    data = json.load(file)
                    data["timestamp"] = timestamp  # add timestamp to JSON 
                    data["node"] = node_label
                    json_data.append(data)
            except (IOError, json.JSONDecodeError) as e:
                print(f"Error reading file: {filepath}. {str(e)}")
    return json_data

def separate_ego_vehicle(actors):
    # Filter out the ego vehicle and return only non-ego vehicles
    return [actor for actor in actors if actor['type'] != 'ego_vehicle']

def groundtruth_for_detection(json_objects):
    object_utils = ObjectUtils()
    # Prepare the list to hold all parsed records
    records = []
    max_non_ego_actors = 0
    # Sort the json_data by timestamp
    json_objects.sort(key=lambda x: x['timestamp'])
    for frame_id, json_data in enumerate(json_objects):
        # Extract the non-ego actors
        non_ego_actors = tf_pose_relative_ego(json_data['actors'])
        max_non_ego_actors = max(max_non_ego_actors, len(non_ego_actors))

        timestamp = json_data['timestamp']
        node = json_data["node"]
        # Initialize a record dictionary with common fields
        record = {'frame_id': frame_id,'timestamp': timestamp, 'node': node, 'objects_num':len(non_ego_actors)}
        # Extract the timestamp and frame_id from the header
        # Iterate through each non-ego actor
        for i, actor in enumerate(non_ego_actors):
            type = object_utils.map_object_type(actor['type'])
            # Add actor-specific information to the record with a unique prefix
            prefix = f'object_{i}_'
            record.update({
                prefix + 'type': type,
                prefix + 'id': i,
                prefix + 'pos_x': actor['pose']['position']['x'],
                prefix + 'pos_y': actor['pose']['position']['y'],
                prefix + 'pos_z': actor['pose']['position']['z'],
                prefix + 'orientation_x': actor['pose']['orientation']['x'],
                prefix + 'orientation_y': actor['pose']['orientation']['y'],
                prefix + 'orientation_z': actor['pose']['orientation']['z'],
                prefix + 'orientation_w': actor['pose']['orientation']['w'],
                prefix + 'boundingbox_x': actor['boundingbox']['x'],
                prefix + 'boundingbox_y': actor['boundingbox']['y'],
                prefix + 'boundingbox_z': actor['boundingbox']['z'],
                prefix + 'distance': actor['distance'],
            })
            # Append the record to the list
        # Fill in the blanks for the frames with fewer non-ego actors than the max
        for j in range(len(non_ego_actors), max_non_ego_actors):
            prefix = f'object_{j}_'
            record.update({
                prefix + 'type': None,
                prefix + 'id': None,
                prefix + 'pos_x': None,
                prefix + 'pos_y': None,
                prefix + 'pos_z': None,
                prefix + 'orientation_x': None,
                prefix + 'orientation_y': None,
                prefix + 'orientation_z': None,
                prefix + 'orientation_w': None,
                prefix + 'boundingbox_x': None,
                prefix + 'boundingbox_y': None,
                prefix + 'boundingbox_z': None,
                prefix + 'distance': None,
            })
        records.append(record)
    # Create a DataFrame from the list of records
    df = pd.DataFrame(records)
    return df

def groundtruth_to_dataframe(json_objects):
    object_utils = ObjectUtils()
    # Prepare the list to hold all parsed records
    records = []
    max_non_ego_actors = 0
    # Sort the json_data by timestamp
    json_objects.sort(key=lambda x: x['timestamp'])
    for frame_id, json_data in enumerate(json_objects):
        # Extract the non-ego actors
        non_ego_actors = separate_ego_vehicle(json_data['actors'])
        max_non_ego_actors = max(max_non_ego_actors, len(non_ego_actors))
        timestamp = json_data['timestamp']
        node = json_data["node"]
        # Initialize a record dictionary with common fields
        record = {'frame_id': frame_id,'timestamp': timestamp, 'node': node, 'objects_num':len(non_ego_actors)}
        # Extract the timestamp and frame_id from the header
        # Iterate through each non-ego actor
        for i, actor in enumerate(non_ego_actors):
            type = object_utils.map_object_type(actor['type'])
            # Add actor-specific information to the record with a unique prefix
            prefix = f'object_{i}_'
            record.update({
                prefix + 'type': type,
                prefix + 'id': i,
                prefix + 'pos_x': actor['pose']['position']['x'],
                prefix + 'pos_y': actor['pose']['position']['y'],
                prefix + 'pos_z': actor['pose']['position']['z'],
                prefix + 'orientation_x': actor['pose']['orientation']['x'],
                prefix + 'orientation_y': actor['pose']['orientation']['y'],
                prefix + 'orientation_z': actor['pose']['orientation']['z'],
                prefix + 'orientation_w': actor['pose']['orientation']['w'],
                prefix + 'boundingbox_x': actor['boundingbox']['x'],
                prefix + 'boundingbox_y': actor['boundingbox']['y'],
                prefix + 'boundingbox_z': actor['boundingbox']['z'],
                prefix + 'distance': actor['distance'],
            })
            # Append the record to the list
        # Fill in the blanks for the frames with fewer non-ego actors than the max
        for j in range(len(non_ego_actors), max_non_ego_actors):
            prefix = f'object_{j}_'
            record.update({
                prefix + 'type': None,
                prefix + 'id': None,
                prefix + 'pos_x': None,
                prefix + 'pos_y': None,
                prefix + 'pos_z': None,
                prefix + 'orientation_x': None,
                prefix + 'orientation_y': None,
                prefix + 'orientation_z': None,
                prefix + 'orientation_w': None,
                prefix + 'boundingbox_x': None,
                prefix + 'boundingbox_y': None,
                prefix + 'boundingbox_z': None,
                prefix + 'distance': None,
            })
        records.append(record)
    # Create a DataFrame from the list of records
    df = pd.DataFrame(records)
    return df

def object_to_dataframe(json_data):
    # Prepare the list to hold all parsed records
    records = []
    
    # First, find the maximum number of objects in any frame
    max_objects = max(len(data['objects']) for data in json_data)

    # Sort the json_data by timestamp
    json_data.sort(key=lambda x: x['timestamp'])
    
    for frame_id, data in enumerate(json_data):
        # Extract the timestamp and node from the data
        timestamp = data['timestamp']
        node = data["node"]
        # Initialize a record dictionary with common fields
        record = {'frame_id': frame_id, 'timestamp': timestamp, 'node': node, 'objects_num': len(data["objects"])}
        
        # Iterate through each actor in the JSON data
        for i, actor in enumerate(data["objects"]):
            # Add actor-specific information to the record with a unique prefix
            prefix = f'object_{i}_'
            record.update({
                prefix + 'type': actor['label'],
                prefix + 'id': i,
                prefix + 'pos_x': actor['pose']['position']['x'],
                prefix + 'pos_y': actor['pose']['position']['y'],
                prefix + 'pos_z': actor['pose']['position']['z'],
                prefix + 'orientation_x': actor['pose']['orientation']['x'],
                prefix + 'orientation_y': actor['pose']['orientation']['y'],
                prefix + 'orientation_z': actor['pose']['orientation']['z'],
                prefix + 'orientation_w': actor['pose']['orientation']['w'],
                prefix + 'boundingbox_x': actor['dimensions']['x'],
                prefix + 'boundingbox_y': actor['dimensions']['y'],
                prefix + 'boundingbox_z': actor['dimensions']['z'],
            })
        
        # For frames with fewer objects, fill in the missing columns with NaN
        for j in range(len(data["objects"]), max_objects):
            prefix = f'object_{j}_'
            record.update({
                prefix + 'type': None,
                prefix + 'id': None,
                prefix + 'pos_x': None,
                prefix + 'pos_y': None,
                prefix + 'pos_z': None,
                prefix + 'orientation_x': None,
                prefix + 'orientation_y': None,
                prefix + 'orientation_z': None,
                prefix + 'orientation_w': None,
                prefix + 'boundingbox_x': None,
                prefix + 'boundingbox_y': None,
                prefix + 'boundingbox_z': None,
            })
        
        # Append the record to the list
        records.append(record)
    
    # Create a DataFrame from the list of records
    df = pd.DataFrame(records)
    return df

def extract_object_info(row, object_index):
    # Assuming each object has properties in the DataFrame like 'object_{index}_id', 'object_{index}_type',
    # 'object_{index}_pos_x', 'object_{index}_pos_y', and 'object_{index}_pos_z'
    object_info = {
        'node': row['node'],
        'id': row[f'object_{object_index}_id'],
        'type': row[f'object_{object_index}_type'],
        'pos_x': row[f'object_{object_index}_pos_x'],
        'pos_y': row[f'object_{object_index}_pos_y'], # Assuming z-coordinate is optional
    }
    return object_info

def extract_groundtruth_info(row, object_index):
    # Assuming each object has properties in the DataFrame like 'object_{index}_id', 'object_{index}_type',
    # 'object_{index}_pos_x', 'object_{index}_pos_y', and 'object_{index}_pos_z'
    object_info = {
        'node': row['node'],
        'id': row[f'object_{object_index}_id'],
        'type': row[f'object_{object_index}_type'],
        'pos_x': row[f'object_{object_index}_pos_x'],
        'pos_y': row[f'object_{object_index}_pos_y'], # Assuming z-coordinate is optional
        'distance': row[f'object_{object_index}_distance'],
    }
    return object_info

def evaluate_detection(groundtruth_df, detection_df, distance_threshold=6, binary =True) -> pd.DataFrame:
    results_list = []

    # Iterate over each frame_id in the ground truth DataFrame
    for frame_id in groundtruth_df['frame_id'].unique():
        frame_results = {
            'frame_id': frame_id,
        }

        # Get the ground truth and detection rows for the current frame_id
        gt_rows = groundtruth_df[groundtruth_df['frame_id'] == frame_id]
        detection_rows = detection_df[detection_df['frame_id'] == frame_id]

        # Extract ground truth and detection objects
        gt_objects = [extract_groundtruth_info(gt_row, i) for _, gt_row in gt_rows.iterrows() for i in range(gt_row['objects_num'])]
        detected_objects = [extract_object_info(det_row, i) for _, det_row in detection_rows.iterrows() for i in range(det_row['objects_num'])]

        # Calculate distances between all ground truth and detected objects if both are non-empty
        if gt_objects and detected_objects:
            gt_positions = [(obj['pos_x'], obj['pos_y']) for obj in gt_objects]
            detected_positions = [(obj['pos_x'], obj['pos_y']) for obj in detected_objects]
            distances = cdist(gt_positions, detected_positions, metric='euclidean')            
            # Create a list to keep track of matched detected objects
            matched_detected_indices = set()
            # Process matches, classification, and localization
            for i, gt_obj in enumerate(gt_objects):
                # Find the closest detected object
                detected_distances = np.copy(distances[i])
                for index in matched_detected_indices:
                    detected_distances[index] = np.inf
                detected_index = np.argmin(detected_distances)
                min_distance = detected_distances[detected_index]
                id = int(gt_obj["distance"])
                if min_distance < distance_threshold:
                    # We have a match, so record the index
                    matched_detected_indices.add(detected_index)
                    detected_obj = detected_objects[detected_index]
                    # Check classification
                    is_wrong_classification = gt_obj['type'] != detected_obj['type']
                    # Check localization
                    # is_wrong_localization = min_distance > (distance_threshold-2)
                    is_wrong_localization = min_distance > 4
                    if is_wrong_classification:
                        frame_results[f'object_{id}_wrong_classification'] = detected_obj
                    if is_wrong_localization:
                        frame_results[f'object_{id}_wrong_localization'] = detected_obj
                else:
                    # No detected object is close enough to the ground truth
                    frame_results[f'object_{id}_false_negative'] = gt_obj                
            # Now handle the unmatched detected objects
            for j, detected_obj in enumerate(detected_objects):
                if j not in matched_detected_indices:
                    j = int(detected_obj["id"])
                    # This detected object was not matched, so it's a false positive
                    frame_results[f'object_{j}_false_positive'] = detected_obj
        else:
            # If there are no detected objects, all ground truth objects are false negatives
            for i, gt_obj in enumerate(gt_objects):
                id = int(gt_obj["distance"])
                frame_results[f'object_{id}_false_negative'] = gt_obj

        results_list.append(frame_results)
        final_results_df = pd.DataFrame(results_list)
        final_results_df = final_results_df.drop(columns=["frame_id"])
    if binary:
        for col in final_results_df.columns:
            final_results_df[col] = final_results_df[col].notnull().astype(int)
        return final_results_df, final_results_df.columns
    else:
        return final_results_df, final_results_df.columns

def roi_consistency(groundtruth_df, roi_df, binary =True) -> pd.DataFrame:
    results_list = []
    # Iterate over each frame_id in the ground truth DataFrame
    for frame_id in groundtruth_df['frame_id'].unique():
        frame_results = {
            'frame_id': frame_id,
        }
        # Get the ground truth and detection rows for the current frame_id
        gt_rows = groundtruth_df[groundtruth_df['frame_id'] == frame_id]
        detection_rows =roi_df[roi_df['frame_id'] == frame_id]
        # Extract ground truth and detection objects
        gt_objects = [extract_groundtruth_info(gt_row, i) for _, gt_row in gt_rows.iterrows() for i in range(gt_row['objects_num'])]
        detected_objects = [extract_object_info(det_row, i) for _, det_row in detection_rows.iterrows() for i in range(det_row['objects_num'])]               
        if not detected_objects:
            for gt_obj in gt_objects:
                    id = int(gt_obj["distance"])
                    frame_results[f'object_{id}_false_negative'] = gt_obj
        results_list.append(frame_results)
    final_results_df = pd.DataFrame(results_list)
    final_results_df = final_results_df.drop(columns=["frame_id"])
    if binary:
        for col in final_results_df.columns:
            final_results_df[col] = final_results_df[col].notnull().astype(int)
        return final_results_df, final_results_df.columns
    else:
        return final_results_df, final_results_df.columns
    
def create_experiment_file(experiment_id):
    # Ensure the Data_dir directory exists
    experiment_path = os.path.join(Data_dir,experiment_id)
    if not os.path.exists(experiment_path):
        os.makedirs(experiment_path)
    return experiment_path

def get_groundtruth(rosbagname):
    node_label =  "groundtruth"
    publish_topic = node_topics.get(node_label)
    actor_directory = get_directory(rosbagname,publish_topic)
    actor_data =  read_json_files(actor_directory,node_label)
    groundtruth_tracking =  groundtruth_to_dataframe(actor_data)
    groundtruth_detection = groundtruth_for_detection(actor_data)
    return groundtruth_tracking, groundtruth_detection

def get_failures(rosbagname,groundtruth):
    node_label ="multi_object_tracker"
    publish_topic = node_topics.get(node_label)
    actor_directory = get_directory(rosbagname,publish_topic)
    actor_data =  read_json_files(actor_directory,node_label)
    detected_dataframe =  object_to_dataframe(actor_data)
    evaluation_results = evaluate_detection(groundtruth, detected_dataframe)
    return evaluation_results

def get_faults(node_label,rosbagname,groundtruth):
    publish_topic = node_topics.get(node_label)
    actor_directory = get_directory(rosbagname,publish_topic)
    actor_data =  read_json_files(actor_directory,node_label)
    detected_dataframe =  object_to_dataframe(actor_data)
    evaluation_results = evaluate_detection(groundtruth, detected_dataframe)
    return evaluation_results

def get_rois_unconsistency(rosbagname,groundtruth):
    node_label ="tensorrt_yolo"
    publish_topic = node_topics.get(node_label)
    actor_directory = get_directory( rosbagname,publish_topic)
    actor_data =  read_json_files(actor_directory,node_label)
    detected_dataframe =  object_to_dataframe(actor_data)
    evaluation_results = roi_consistency(groundtruth, detected_dataframe)
    return evaluation_results

def save_target_failure(target_objects,groundtruth_detection,rosbagname,bagname,
                   detection_nodes,evaluation_failures, exp_id):
    pattern = r'object_(\d+)_{}'.format(failure_mode)
    fault_modes = ['wrong_classification', 'false_negative', 'wrong_localization',"false_positive"]
    f_pattern = re.compile(pattern)
    # Extracting object IDs that have the false_negative fault mode
    object_ids = set(f_pattern.search(index).group(1) 
                            for index in target_objects if f_pattern.search(index))
        # Check if object_ids_with_fn is not null (empty)
    if not object_ids:
        # Handle the case when object_ids_with_fn is empty
        print(f"No objects found with failure mode: {failure_mode}")
        return
    for object_id in object_ids:
        failure= evaluation_failures[f'object_{object_id}_{failure_mode}']
        data = pd.DataFrame()
        data[f'multi_object_tracker_{failure_mode}'] = failure
        for node_label in detection_nodes:
            faults,fault_objects = get_faults(node_label, rosbagname,groundtruth_detection)
            for mode in fault_modes:
                fault_mode_object = f'object_{object_id}_{mode}'
                if fault_mode_object in fault_objects:
                    data[f'{node_label}_{mode}'] = faults[fault_mode_object]
        node_label = "tensorrt_yolo"
        roi, roi_objects = get_rois_unconsistency(rosbagname,groundtruth_detection)
        for mode in fault_modes:
                fault_mode_object = f'object_{object_id}_{mode}'
                if fault_mode_object in roi_objects:
                    data[f'{node_label}_{mode}'] = roi[fault_mode_object]
                    # Define the file path
        file_name = f'{bagname}_object_{object_id}_{failure_mode}.csv'
        exp_path = create_experiment_file(exp_id)
        file_path = os.path.join(exp_path, file_name)
        data.to_csv(file_path, index=False)
        return object_id
 
def search_targets(object_id, evaluation_failures,target_objects,rosbagname,
                    groundtruth_detection,exp_id,bagname):
    target_failure =  f'object_{object_id}_{failure_mode}'
    data = pd.DataFrame()
    if target_failure not in target_objects:
        print(f"No data found for object ID {object_id} with failure mode: {failure_mode}")
    else:
        failure= evaluation_failures[target_failure]
        data[f'multi_object_tracker_{failure_mode}'] = failure   
    fault_modes = ['wrong_classification', 'false_negative', 'wrong_localization',"false_positive"]
    for node_label in detection_nodes:
        faults,fault_objects = get_faults(node_label, rosbagname,groundtruth_detection)
        for mode in fault_modes:
            fault_mode_object = f'object_{object_id}_{mode}'
            if fault_mode_object in fault_objects:
                data[f'{node_label}_{mode}'] = faults[fault_mode_object]
    node_label = "tensorrt_yolo"
    roi, roi_objects = get_rois_unconsistency(rosbagname,groundtruth_detection)
    for mode in fault_modes:
            fault_mode_object = f'object_{object_id}_{mode}'
            if fault_mode_object in roi_objects:
                data[f'{node_label}_{mode}'] = roi[fault_mode_object]
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
    record_id= save_target_failure(target_objects,groundtruth_detection,rosbagname,bagname,
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
    search_targets(object_id, evaluation_failures,target_objects,rosbagname, 
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