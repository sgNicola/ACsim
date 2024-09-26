from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from rosbags.typesys import get_types_from_idl, register_types
from pathlib import Path
from os import listdir
 
import sys
sys.path.append('/home/anonymous/carla_autoware/perceptionIdentify/')
 
from perceptionIdentify.tools.msg_serialize import MessageSerialize
# ==============================================================================
# -- TrackedObjects-------------------------------------------------------------
# ==============================================================================  
def get_tracked_objects(objects_msg):
    tracked_objects = []
    header ={}
    frame_object={}
    frame_id = objects_msg.header.frame_id
    sec = objects_msg.header.stamp.sec
    nanosec = objects_msg.header.stamp.sec
    stamp ={"sec":sec, "nanosec":nanosec}
    header ={"stamp":stamp,"frame_id":frame_id}
    id = 0
    for object_ in objects_msg.objects:
        pose = object_.kinematics.pose_with_covariance.pose
        pose= MessageSerialize.pose_serialize(pose)
        twist = MessageSerialize.twist_serialize(object_.kinematics.twist_with_covariance.twist)
        accel = MessageSerialize.accel_serialize(object_.kinematics.acceleration_with_covariance.accel)
        pose_covariance = object_.kinematics.pose_with_covariance.covariance.tolist() 
        accel_covariance = object_.kinematics.acceleration_with_covariance.covariance.tolist()
        twist_covariance = object_.kinematics.twist_with_covariance.covariance.tolist()
        orientation_availability= object_.kinematics.orientation_availability
        is_stationary = object_.kinematics.is_stationary
        shape_type = object_.shape.type
        shape_footpoint = MessageSerialize.geometry_polygon_serialize(object_.shape.footprint)
        dimensions = MessageSerialize.boundingbox_serialize(object_.shape.dimensions)
        for classification in object_.classification:
            label = classification.label
        
        tracked_object = {'label':label,
                          "id": id,
                          'pose':pose,
                          "pose_covariance" : pose_covariance,
                          "twist":twist,
                          "twist_covariance":twist_covariance,
                          "orientation_availability":orientation_availability,
                            "accel":accel,
                            "accel_covariance":accel_covariance,
                              "is_stationary":is_stationary,
                              "shape_type": shape_type,
                              "shape_footpoint": shape_footpoint,
                              "dimensions": dimensions,}
        id = id+1
        tracked_objects.append(tracked_object)
    frame_object ={"header":header, "objects":tracked_objects}
    return frame_object

# ==============================================================================
# -- CarlaObjects-------------------------------------------------------------
# ==============================================================================

def get_actors(objects_msg):
    actors =[]
    header ={}
    frame_object={}
    frame_id = objects_msg.header.frame_id
    sec = objects_msg.header.stamp.sec
    nanosec = objects_msg.header.stamp.sec
    stamp ={"sec":sec, "nanosec":nanosec}
    header ={"stamp":stamp,"frame_id":frame_id}
    for object_ in objects_msg.actors:
        carla_actor_type = object_.type
        carla_actor_id = object_.id
        actor_current_ros_pose = MessageSerialize.pose_serialize(object_.pose)
        actor_current_ros_twist_rotated =  MessageSerialize.twist_serialize(object_.twist)
        actor_current_ros_accel = MessageSerialize.accel_serialize(object_.accel)
        actor_extent = MessageSerialize.boundingbox_serialize(object_.boundingbox)
        dist = object_.distance
        object ={
                        "type":carla_actor_type,
                        "id": int(carla_actor_id),
                        "pose": actor_current_ros_pose,
                        "twist": actor_current_ros_twist_rotated,
                        "accel": actor_current_ros_accel,
                        "boundingbox":  actor_extent,
                        "distance": dist
                    }    
        actors.append(object)
    frame_object ={"header":header, "actors":actors}
    return frame_object

# ==============================================================================
# -- DetectObjects-------------------------------------------------------------
# ==============================================================================
def get_detected_objects(objects_msg):
    detected_objects = []
    header ={}
    frame_object={}
    frame_id = objects_msg.header.frame_id
    sec = objects_msg.header.stamp.sec
    nanosec = objects_msg.header.stamp.sec
    stamp ={"sec":sec, "nanosec":nanosec}
    header ={"stamp":stamp,"frame_id":frame_id}
    id = 0
    for object_ in objects_msg.objects:
        pose = object_.kinematics.pose_with_covariance.pose
        pose= MessageSerialize.pose_serialize(pose)
        twist = MessageSerialize.twist_serialize(object_.kinematics.twist_with_covariance.twist)
        pose_covariance = object_.kinematics.pose_with_covariance.covariance.tolist()  
        twist_covariance = object_.kinematics.twist_with_covariance.covariance.tolist()
        orientation_availability= object_.kinematics.orientation_availability
        shape_type = object_.shape.type
        shape_footpoint = MessageSerialize.geometry_polygon_serialize(object_.shape.footprint)
        dimensions = MessageSerialize.boundingbox_serialize(object_.shape.dimensions)
        for classification in object_.classification:
            label = classification.label
        
        detected_object = {'label':label,
                          "id": id,
                          'pose':pose,
                          "pose_covariance" : pose_covariance,
                          "twist":twist,
                          "twist_covariance":twist_covariance,
                          "orientation_availability":orientation_availability,
                              "shape_type": shape_type,
                              "shape_footpoint": shape_footpoint,
                              "dimensions": dimensions,}
        id = id+1
        detected_objects.append(detected_object)
    frame_object ={"header":header, "objects":detected_objects}
    return frame_object


# ==============================================================================
# -- DetectObjectswithFeatures-------------------------------------------------- 
# ==============================================================================
def get_detectObjectswithFeatures(objects_msg):
    detected_objects = []
    header ={}
    frame_object={}
    frame_id = objects_msg.header.frame_id
    sec = objects_msg.header.stamp.sec
    nanosec = objects_msg.header.stamp.sec
    stamp ={"sec":sec, "nanosec":nanosec}
    header ={"stamp":stamp,"frame_id":frame_id}
    id = 0
    for detectedObjectWithFeature in objects_msg.feature_objects:
        object_ = detectedObjectWithFeature.object
        pose = object_.kinematics.pose_with_covariance.pose
        pose= MessageSerialize.pose_serialize(pose)
        twist = MessageSerialize.twist_serialize(object_.kinematics.twist_with_covariance.twist)
        pose_covariance = object_.kinematics.pose_with_covariance.covariance.tolist()  
        twist_covariance = object_.kinematics.twist_with_covariance.covariance.tolist()
        orientation_availability= object_.kinematics.orientation_availability
        shape_type = object_.shape.type
        shape_footpoint = MessageSerialize.geometry_polygon_serialize(object_.shape.footprint)
        dimensions = MessageSerialize.boundingbox_serialize(object_.shape.dimensions)
        # feature_data = detectedObjectWithFeature.feature.cluster.data.tolist()
        for classification in object_.classification:
            label = classification.label
        roi = MessageSerialize.sensor_roi_serialize(detectedObjectWithFeature.feature.roi)
        cluster = MessageSerialize.pointcloud2_serialize(detectedObjectWithFeature.feature.cluster)
        detected_object = {'label':label,
                          "id": id,
                          'pose':pose,
                          "pose_covariance" : pose_covariance,
                          "twist":twist,
                          "twist_covariance":twist_covariance,
                          "orientation_availability":orientation_availability,
                              "shape_type": shape_type,
                              "shape_footpoint": shape_footpoint,
                              "dimensions": dimensions,
                              "roi": roi,
                               }
        id = id+1
        detected_objects.append(detected_object)
    frame_object ={"header":header, "objects":detected_objects}
    return frame_object
# ==============================================================================
# -- EmptyMsg-------------------------------------------------- 
# =====================================================================
def empty_msg(objects_msg):
    actors =[]
    header ={}
    frame_object={}
    frame_id = objects_msg.header.frame_id
    sec = objects_msg.header.stamp.sec
    nanosec = objects_msg.header.stamp.sec
    stamp ={"sec":sec, "nanosec":nanosec}
    header ={"stamp":stamp,"frame_id":frame_id}
    frame_object ={"header":header, "actors":actors}
    return frame_object