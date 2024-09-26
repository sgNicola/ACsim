from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from rosbags.typesys import get_types_from_idl, register_types
from pathlib import Path
import os
from os import listdir
import re
import json
import param
def add_msg_types(idl_paths):
    for idlpath in idl_paths:
        filenames = [ f for f in listdir(idlpath)]
        add_types = {}
        for pathstr in filenames:
            pathstr =os.path.join(idlpath,pathstr)
            msgpath = Path(pathstr)
            msgdef = msgpath.read_text(encoding='utf-8')
            add_types.update(get_types_from_idl(msgdef))
        register_types(add_types)
    return 

def get_folder_names(directory):
        folder_names = [name for name in os.listdir(directory) if os.path.isdir(os.path.join(directory, name))]
        return  folder_names

class ExtractTopic(object):

    def __init__(self):
        super().__init__()
        self.Ros_dir = param.Rosbag_dir  
        self.ObjectData_dir = param.ObjectData_dir

    def get_time_msg(self, rosbagname,topic):
        # input: 
        # return: 1. time stamp
        #         2. messages
        rosbag_path=os.path.join(self.Ros_dir, rosbagname)
        timestamps =[]
        msgs =[]
        with Reader(rosbag_path) as reader:
            connections = [x for x in reader.connections if x.topic == topic]
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = deserialize_cdr(rawdata, connection.msgtype)
                msgs.append(msg)
                timestamps.append(timestamp)
            # print(connection.msgtype)
        timestamp_msgs={"time":timestamps, "msg": msgs}
        return timestamp_msgs

    def object_topics(self,rosbagname):
        rosbag_path=os.path.join(self.Ros_dir, rosbagname)
        topics=[]
        msgtypes=[]
        with Reader(rosbag_path) as reader:
            for connection in reader.connections:
                topics.append(connection.topic)
                msgtypes.append(connection.msgtype)
        # return list of different objects
        pattern = r'/perception.*/objects'
        perception_topics = [item for item in topics if re.match(pattern, item)]
        prediction_objects=['/perception/object_recognition/objects']
        detection_object_topics = [item for item in perception_topics if "detection" in item]
        tracking_object_topics = [item for item in perception_topics if "tracking" in item]
        return perception_topics, prediction_objects,detection_object_topics,tracking_object_topics

    def create_topic_folders(self, topics, rosbagname):
    # topics: perception_topics
    # base_directory:  RosbagName_dir
        base_directory  =  self.ObjectData_dir+rosbagname
        for topic in topics:
            topic_folder = topic.replace("/","_")
            folder_path = os.path.join(base_directory, topic_folder)
            if not os.path.exists(folder_path):
                os.makedirs(folder_path)
        return 
    
    def dump_data_to_json(self, data, timestamp,rosbagname,topic):  
        # base_directory = topic_dir = RosbagName_dir + topic_name
        # store the object data in a folder, each time stamp for a frame
        RosbagName_dir = self.ObjectData_dir+rosbagname
        topic_folder = topic.replace("/","_")
        base_directory  = os.path.join(RosbagName_dir,topic_folder)
        if not os.path.exists(base_directory):
            os.makedirs(base_directory)
        filename = f"{timestamp}.json"  
        file =os.path.join(base_directory,filename)
        with open(file, 'w') as file:
            json.dump(data, file, indent=4, ensure_ascii=False)
        return