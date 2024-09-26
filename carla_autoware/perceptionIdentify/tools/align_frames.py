import numpy as np
from perceptionIdentify.tools.readmsg_agent import ExtractTopic

# ==============================================================================
# --  align_groundtruth_basetimestamp-------------------------------------------
# ==============================================================================

def align_timestamps(timestamps_a, timestamps_b):
    if isinstance(timestamps_b, np.ndarray):
        timestamps_b =  timestamps_b.tolist()
    else:
        timestamps_b = timestamps_b
    
    start_time = max(timestamps_a[0], timestamps_b[0])
    closest_timestamp_a = min(timestamps_a, key = lambda x : abs(x - start_time))
    closest_timestamp_b = min(timestamps_b, key = lambda x : abs(x - start_time))

    end_time = min(timestamps_a[-1], timestamps_b[-1])
    end_timestamp_a = min(timestamps_a, key = lambda x : abs(x - end_time))
    end_timestamp_b = min(timestamps_b, key = lambda x : abs(x - end_time))
    
    start_index_a = timestamps_a.index(closest_timestamp_a)
    start_index_b = timestamps_b.index(closest_timestamp_b)

    end_index_a = timestamps_a.index(end_timestamp_a)
    end_index_b = timestamps_b.index(end_timestamp_b)

    timestamps_a = timestamps_a[start_index_a:end_index_a]
    timestamps_b =  timestamps_b[start_index_b:end_index_b]
    # find the minist list as the basetimestamp 
    base_timestamps = timestamps_a if len(timestamps_a) < len(timestamps_b) else timestamps_b
    target_timestamps = timestamps_a if base_timestamps is timestamps_b else timestamps_b
    base_timestamps = np.array(base_timestamps)
    target_timestamps = np.array(target_timestamps)
 
    aligned_timestamps = []
    
    for timestamp in base_timestamps:
        # Find the index of the closest timestamp in the target list
        index = np.argmin(np.abs(target_timestamps - timestamp))
        
        # Assign the corresponding timestamp to the aligned_timestamps array
        aligned_timestamps.append(target_timestamps[index])
    
    return aligned_timestamps, base_timestamps


# ==============================================================================
# --  align_detectedobject_basetimestamp----------------------------------------
# ==============================================================================

def align_detected_timestamps(detectedobject_msgs, base_timestamps):
    base_timestamps = np.array(base_timestamps)
    target_timestamps = detectedobject_msgs["time"]
    target_timestamps = np.array(target_timestamps)
    filtered_msgs = []
    for i, timestamp in enumerate(base_timestamps):
            # Find the index of the closest timestamp in the target list
            index = np.argmin(np.abs(target_timestamps - timestamp))
            # Assign the corresponding timestamp to the aligned_timestamps array
            # aligned_timestamp = target_timestamps[index]
            filtered_msgs.append({"time": target_timestamps[index] , "msg":detectedobject_msgs["msg"][index]})

    return filtered_msgs

# def align_detected_timestamps(target_timestamps, base_timestamps):
#     base_timestamps = np.array(base_timestamps)
#     target_timestamps = np.array(target_timestamps)
#     # Initialize aligned_timestamps as a NumPy array with the same length as base_timestamps
#     aligned_timestamps = np.zeros_like(base_timestamps)
#     for i, timestamp in enumerate(base_timestamps):
#             # Find the index of the closest timestamp in the target list
#             index = np.argmin(np.abs(target_timestamps - timestamp))
#             # Assign the corresponding timestamp to the aligned_timestamps array
#             aligned_timestamps[i] = target_timestamps[index]
        
#     return aligned_timestamps

def filter_timestamps(timestamp_msgs, timestamps):
    filtered_msgs = []
    for i in range(len(timestamp_msgs["time"])):
        if timestamp_msgs["time"][i] in timestamps:
            filtered_msgs.append({"time": timestamp_msgs["time"][i], "msg": timestamp_msgs["msg"][i]})
    return filtered_msgs

def save_filtered_frames(filtered_msg, rosbagname, topic, extract_func):
    # save filtered msg in json
    extracttopic = ExtractTopic()
    for msg in filtered_msg:
        frame_objects = extract_func(msg["msg"])
        ts = msg["time"]
        extracttopic.dump_data_to_json(frame_objects, ts, rosbagname, topic)