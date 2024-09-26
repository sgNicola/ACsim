from math import ceil
import os
from param import Data_dir, failure_mode
import pandas as pd
from perceptionIdentify.process_rosbag import process_rosbag_file
# from run_lidar import run_carla_scenario_agent
from run_fusion import run_carla_scenario_agent
import time
from pandas.errors import EmptyDataError
from perceptionIdentify.parse_fusion import record_intervene
# from perceptionIdentify.parse_lidar import record_intervene
from param import Rosbag_dir
threshold = 0.5  # 50% threshold

def get_non_zero_nodes(df):
    """
    Identify nodes with non-zero values in the DataFrame.
    
    Parameters:
    - df: A pandas DataFrame where columns represent nodes and rows represent observations or states.
    
    Returns:
    - non_zero_nodes: A list of nodes with non-zero values in the DataFrame.
    """
    # non_zero_nodes = df.columns[(df != 0).any()].tolist()
    threshold = 0.5
    variables = df.columns[(df != 0).mean() >= threshold].tolist()
    return variables

def get_first_half(V):
    """
    Get the first half of the provided nodes in topological order.

    Parameters:
    V (List): A list of nodes in topological order.
    Returns:
    List: The first half of the provided nodes.
    """
    # Calculate the index to split the list in half (using ceil to round up for odd lengths)
    half_index = ceil(len(V) / 2)    
    # Return the first half of the nodes
    return V[:half_index]

# def read_failure(file_name):
#     try:
#         df = pd.read_csv(file_name)
#         if df.empty:
#             return df
#         # List of node labels
#         node_labels = [
#             'lidar_centerpoint',
#             'multi_object_tracker',
#             'detected_object_feature_remover',
#             'roi_cluster_fusion',
#             'fusion_shape_estimation',
#             'clustering_shape_estimation',
#             'euclidean_cluster',
#             'object_association_merger_0',
#             'object_association_merger_1',
#             'obstacle_pointcloud_based_validator',
#             'tensorrt_yolo']

#         # Initialize a list to keep track of columns to be dropped after processing
#         columns_to_drop = []
#         # Loop through each node label and sum the respective columns
#         for node in node_labels:
#             wrong_classification_col = f"{node}_wrong_classification"
#             false_negative_col = f"{node}_false_negative"
#             wrong_localization_col = f"{node}_wrong_localization"
#             false_positive_col = f"{node}_false_positive"
#             # Sum the columns if they exist in the DataFrame
#             for col in [wrong_classification_col, false_negative_col, wrong_localization_col, false_positive_col]:
#                 if col in df.columns:
#                     # Initialize the node column with the first valid column found or add to it
#                     df[node] = df[node] + df[col] if node in df.columns else df[col] 
#                     columns_to_drop.append(col)
#         # Drop the processed columns
#         df.drop(columns_to_drop, axis=1, inplace=True)
#         # Drop any columns where all values are zero
#         df = df.loc[:, (df != 0).any(axis=0)]
#         return df
#     except EmptyDataError:
#         print(f"No data in file {file_name}.")
#         return pd.DataFrame()
    
def read_failure(file_name):
    try:
        df = pd.read_csv(file_name)
        if df.empty:
            return df
        # List of node labels
        node_labels = [
            'lidar_centerpoint',
            'multi_object_tracker',
            'detected_object_feature_remover',
            'roi_cluster_fusion',
            'fusion_shape_estimation',
            'clustering_shape_estimation',
            'euclidean_cluster',
            'object_association_merger_0',
            'object_association_merger_1',
            'obstacle_pointcloud_based_validator',
            'tensorrt_yolo']

        # Initialize a list to keep track of columns to be dropped after processing
        columns_to_drop = []
        for node in node_labels:
            false_negative_col = f"{node}_false_negative"
            false_positive_col = f"{node}_false_positive"
            wrong_classification_col = f"{node}_wrong_classification"
            wrong_localization_col = f"{node}_wrong_localization"    
            # encode the columns if they exist in the DataFrame
            # Reverse the order as binary digits are from right to left (least significant to most significant)
            cols_to_encode =[false_negative_col,false_positive_col, wrong_classification_col, wrong_localization_col]
            for col in reversed(cols_to_encode):
                if col in df.columns:
                    # Initialize the node column with the first valid column found or add to it
                    df[node] = df[node]*2 + df[col] if node in df.columns else df[col] 
                    columns_to_drop.append(col)
        # Drop the processed columns
        df.drop(columns_to_drop, axis=1, inplace=True)
        # Drop any columns where all values are zero
        df = df.loc[:, (df != 0).any(axis=0)]
        return df
    except EmptyDataError:
        print(f"No data in file {file_name}.")
        return pd.DataFrame()
    
def column_with_most_non_zeros(dataframe, keyword):
    # Filter columns that contain the keyword
    matching_columns = [col for col in dataframe.columns if keyword in col]
    # Initialize max count and column name holder
    max_non_zero_count = 0
    column_name_with_max_non_zeros = None
    # Iterate over matching columns to find the one with the most non-zero values
    for column in matching_columns:
        non_zero_count = dataframe[column].astype(bool).sum()  # Count non-zero entries
        if non_zero_count > max_non_zero_count:
            max_non_zero_count = non_zero_count
            column_name_with_max_non_zeros = column
    return column_name_with_max_non_zeros
    
def decode_fault_mode(experiment_id,object_id,node_labels):
    # node_labels are the causal modules, eg. C
    file_name = f'01_object_{object_id}_{failure_mode}.csv'
    file_id = os.path.join(experiment_id,file_name)
    file = os.path.join(Data_dir,file_id)
    try:
        df = pd.read_csv(file)
        columns_with_most_non_zeros = {keyword: column_with_most_non_zeros(df, keyword) for keyword in node_labels}
        return columns_with_most_non_zeros
    except EmptyDataError:
        print(f"No data in file {file_name}.")

def select_max_overlap_column(data, node_label, target_failure):
    """
    param:
        file_path (str): CSV 
        node_label (str): 
        target_node (str):
        
    return:
        str: column name。
    """
    target_col =f"multi_object_tracker_{target_failure}"
    if target_col not in data.columns:
        raise ValueError(f"The target column '{target_col}' does not exist in the data.")
    matched_columns = [col for col in data.columns if node_label in col]
    overlap_counts = {col: (data[col] & data[target_col]).sum() for col in matched_columns}
    max_overlap_column = max(overlap_counts, key=overlap_counts.get)
    return max_overlap_column
 
def get_causals(experiment_id,object_id,bag_id):
    file_name = f'{bag_id}_object_{object_id}_{failure_mode}.csv'
    file_id = os.path.join(experiment_id,file_name)
    file = os.path.join(Data_dir,file_id)
    df = read_failure(file)
    variables = df.columns[(df != 0).mean() > threshold].tolist()
    return variables

def run_and_process_rosbag(experiment_id,object_id,bag_id,scenario,params,targets):
    ros_bag = os.path.join(experiment_id, bag_id)
    ROS_file = os.path.join(Rosbag_dir,ros_bag)
    if not os.path.exists(ROS_file):
        try:
            run_carla_scenario_agent(experiment_id, bag_id,scenario,params,targets)
        except Exception as e:
            print(f"Error running carla_scenario_agent for experiment ID '{experiment_id}': {str(e)}")
            time.sleep(10)
            run_carla_scenario_agent(experiment_id, bag_id,scenario, params,targets)
    try:
        process_rosbag_file(ros_bag)
        print(f"processed rosbag '{ros_bag}")
    except Exception as e:
        print(f"Error processing rosbag '{ros_bag}': {str(e)}")
        run_carla_scenario_agent(experiment_id, ros_bag,scenario,params,targets)
        time.sleep(1)
        process_rosbag_file(ros_bag)
    record_intervene(object_id, bag_id, experiment_id)
    file_name = f'{bag_id}_object_{object_id}_{failure_mode}.csv'
    file_id = os.path.join(experiment_id,file_name)
    file = os.path.join(Data_dir,file_id)
    df = read_failure(file)
    variables = df.columns[(df != 0).mean() > threshold].tolist()
    return variables