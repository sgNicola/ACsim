from perceptionIdentify.tools.readmsg_agent import ExtractTopic, get_folder_names, add_msg_types
from perceptionIdentify.tools.align_frames import align_timestamps, filter_timestamps, save_filtered_frames, align_detected_timestamps 
from perceptionIdentify.serialize_objects import get_actors, get_detected_objects, get_tracked_objects, get_detectObjectswithFeatures,empty_msg
import yaml
def load_config(config_file):
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    return config

def generate_empty_msg(timestamp_ns):
        # Create the empty message structure
        # Construct the header
    sec = timestamp_ns // 1_000_000_000
    nanosec = timestamp_ns % 1_000_000_000
    header = {
        "stamp": {
            "sec": sec,
            "nanosec":nanosec,
        },
        "frame_id":"base_link"
    }
    empty_msg = {
        "header":  header,
        "objects": []
    }
    return empty_msg

def save_filter_detection_frames(rosbagname, detected_object_topic, get_detected_objects, base_timestamps):
    extracttopic = ExtractTopic()
    detectedobject_msgs = extracttopic.get_time_msg(rosbagname, detected_object_topic)
    detectedobject_msg = align_detected_timestamps(detectedobject_msgs, base_timestamps)
    save_filtered_frames(detectedobject_msg, rosbagname, detected_object_topic, get_detected_objects)

def process_rosbag_file(rosbagname):
    idl_paths = [
    "/home/anonymous/ACsim/autoware_auto_msgs/autoware_auto_perception_msgs/msg/",
    "/home/anonymous/ACsim/autoware_auto_msgs/carla_msg/msg/",
    "/home/anonymous/ACsim/autoware_auto_msgs/tier4_perception_msgs/msg/"
    ]
    extracttopic = ExtractTopic()
    add_msg_types(idl_paths)
    # perception_topics, prediction_topics, detection_object_topics, tracking_object_topics = extracttopic.object_topics(rosbagname)
    tracking_object_topic = "/perception/object_recognition/tracking/objects"
    ground_truth_topic = "/carla_actors"
    config = load_config('/home/anonymous/ACsim/carla_autoware/perceptionIdentify/fusion_config.yaml')
    # config = load_config('/home/anonymous/ACsim/carla_autoware/perceptionIdentify/lidar_config.yaml')
    groundtruth_msgs = extracttopic.get_time_msg(rosbagname, ground_truth_topic)
    groundtruth_timestamps = groundtruth_msgs["time"]

    trackingobject_msg = extracttopic.get_time_msg(rosbagname, tracking_object_topic)
    trackedobjects_timestamps = trackingobject_msg["time"]
    aligned_timestamps, base_timestamps = align_timestamps(groundtruth_timestamps, trackedobjects_timestamps)

    actor_msg = filter_timestamps(groundtruth_msgs, aligned_timestamps)
    trackedobject_msg = filter_timestamps(trackingobject_msg, base_timestamps)

    save_filtered_frames(actor_msg, rosbagname, ground_truth_topic, get_actors)
    save_filtered_frames(trackedobject_msg, rosbagname, tracking_object_topic, get_tracked_objects)

    objects_withfeature_topics = config['objects_withfeature_topics']

    detected_object_topics = config['detected_object_topics']

    for detected_object_topic in objects_withfeature_topics:
        try:
            save_filter_detection_frames(rosbagname, detected_object_topic, get_detectObjectswithFeatures, aligned_timestamps)
        except Exception as e:
            print(f"Saving EMPTY topic '{detected_object_topic}':{str(e)}")
            save_filtered_frames(actor_msg, rosbagname, ground_truth_topic, empty_msg)

    for detected_object_topic in detected_object_topics:
        try:
            save_filter_detection_frames(rosbagname, detected_object_topic, get_detected_objects, aligned_timestamps)
        except Exception as e:
             print(f"Saving EMPTY topic '{detected_object_topic}':{str(e)}")
             save_filtered_frames(actor_msg, rosbagname, ground_truth_topic, empty_msg)

if __name__ == '__main__':
    rosbagname = "1/01"
    # Create a progress bar with the total number of rosbags
    try:
        process_rosbag_file(rosbagname)
    except Exception as e:
        print(f"Error processing rosbag '{rosbagname}': {str(e)}")