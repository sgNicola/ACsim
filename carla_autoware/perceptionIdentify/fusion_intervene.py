import sys
sys.path.append("")
import perceptionIdentify.tools.transforms as trans
import carla
import numpy as np
import random
import math
from os.path import isfile, join
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped, TwistWithCovariance, TwistStamped, TwistWithCovarianceStamped, Pose  
from std_msgs.msg import Header, String
from autoware_auto_perception_msgs.msg import TrackedObjects, DetectedObject, DetectedObjectKinematics, DetectedObjects, TrackedObject, ObjectClassification
from tier4_perception_msgs.msg import DetectedObjectsWithFeature, DetectedObjectWithFeature
from perceptionIdentify.serialize_objects import get_actors
from builtin_interfaces.msg import Time
from perceptionIdentify.intervention.utilities import ObjectUtils
from autoware_auto_perception_msgs.msg import ObjectClassification
from rclpy.executors import MultiThreadedExecutor
import argparse

def get_distance(ego_vehicle, actor):
    distance_vector = actor.get_transform().location  - ego_vehicle.get_transform().location
    distance = math.sqrt(math.pow(distance_vector.x, 2) + math.pow(distance_vector.y, 2))
    return distance

def process_actor(actor, ego_transform_matrix):
    try:
        actor_pose = trans.carla_transform_to_ros_pose(actor.get_transform())
        actor_twist = trans.carla_velocity_to_ros_twist(actor.get_velocity(), actor.get_angular_velocity())
        actor_accel = trans.carla_acceleration_to_ros_accel(actor.get_acceleration())
        actor_extent = trans.carla_vector_to_ros_vector_rotated(actor.bounding_box.extent, actor.bounding_box.rotation)
        obstacle_transform_matrix = trans.ros_pose_to_transform_matrix(actor_pose)
        relative_transform_matrix = np.matrix(ego_transform_matrix).getI() * np.matrix(obstacle_transform_matrix)
        obstacle_ros_pose = trans.transform_matrix_to_ros_pose(relative_transform_matrix)
        return {
            "type": actor.type_id,
            "id": actor.id,
            "pose":  actor_pose,
            "twist": actor_twist,
            "accel": actor_accel,
            "boundingbox": actor_extent,
            "obstacle_ros_pose": obstacle_ros_pose
        }
    except Exception as e:
        print(f"Error processing actor {actor.id}: {e}")
        return None
        
class GroundTruthListener(Node):

    def __init__(self,node_name, topic_name, callback_name, msg_type):
        super().__init__(node_name) 

        self.itv_publisher = self.create_publisher(msg_type, topic_name, 20) 
        self.timestamp = 0
        timer_period = 0.05  # seconds
        # Use getattr to get the method corresponding to callback_name
        timer_callback = getattr(self, callback_name, None)
        if not callable(timer_callback):
            raise ValueError(f"Callback method named '{callback_name}' not found in {self.__class__.__name__}")
        self.timer = self.create_timer(timer_period, timer_callback)
        self._port =2000
        self._local_host  = "localhost"
        self._frame_rate = 20
        self.client = carla.Client(self._local_host, self._port)
        self.world = self.client.get_world()
        if self.world is not None:
            settings = self.world.get_settings()
            settings.fixed_delta_seconds = 1.0 / self._frame_rate
            settings.synchronous_mode = True
            self.world.apply_settings(settings)


    def get_actor_data(self):
        # Fetch actor data from the CARLA simulator
        bb_save_radius = 50
        if self.world is None:
            raise ValueError("Unable to connect to the CARLA server or get the world object.")
        vehicle_actors =  self.world.get_actors().filter('*vehicle*')
        walker_actors =  self.world.get_actors().filter('*walker*')
        static_actors = self.world.get_actors().filter('*static*')

        # Separate the ego vehicle from other actors
 
        ego_vehicle = None
        other_actors = []

        for actor in vehicle_actors:
            if actor.attributes.get('role_name') == "hero":
                ego_vehicle = actor
            else:
                other_actors.append(actor)
        # Check if ego_vehicle was found
        if ego_vehicle is None:
            raise ValueError("Ego vehicle (hero) not found among actors")
        ego_pose = trans.carla_transform_to_ros_pose(ego_vehicle.get_transform())
        ego_transform_matrix = trans.ros_pose_to_transform_matrix(ego_pose)

        for walker in walker_actors:
            other_actors.append(walker)
        for statics in static_actors:
            other_actors.append(statics)

        # Initialize a list to store data about actors within the save radius
        actor_data_within_radius = []
        for actor in other_actors:
            distance = ego_vehicle.get_location().distance(actor.get_location())
            if distance < bb_save_radius:
                # Only consider actors that are not the ego vehicle and within save radius
                bounding_box_location = actor.bounding_box.location
                if not math.isnan(bounding_box_location.x):
                    processed_obj = process_actor(actor, ego_transform_matrix)
                    actor_data_within_radius.append(processed_obj)
        return actor_data_within_radius

    def get_header(self):
        """
        Returns ROS message header
        """
        header = Header()
        snapshot = self.world.get_snapshot()
        if snapshot:
            self.timestamp = snapshot.timestamp.elapsed_seconds
        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)    

        #print('Sensor Time Stamp: ', header.stamp)    
        return header
        
    def tracked_object(self):

        output_msg = TrackedObjects()
        output_msg.header = self.get_header()
        output_msg.header.frame_id = "map"
        object_utils = ObjectUtils()
        actors = self.get_actor_data()
        for actor_data in actors:
            object  = TrackedObject()
            object.object_id.uuid = ObjectUtils.generate_uuid_as_uint8_array()

            object.existence_probability= 1.0
            """
            return a map function 
            """
            object_id = actor_data["type"]
            obj_classification = ObjectClassification()
            obj_classification.label = object_utils.map_object_type(object_id)
            obj_classification.probability = 1.0
            object.classification.append(obj_classification)

            object.kinematics.pose_with_covariance.pose = actor_data["pose"]
            object.kinematics.pose_with_covariance.pose.position.z += 1
            linear_variance = (0.0, 0.0, 0.0)
            angular_variance = (0.0, 0.0, 0.0)
            pose_covariance = object_utils.generate_twist_with_covariance_covariance(linear_variance, angular_variance)
            object.kinematics.pose_with_covariance.covariance = pose_covariance

            object.kinematics.orientation_availability = 1

            object.kinematics.twist_with_covariance.twist = actor_data["twist"]
            twist_covariance = object_utils.generate_covariance(linear_variance, angular_variance)
            object.kinematics.twist_with_covariance.covariance = twist_covariance 

            object.kinematics.acceleration_with_covariance.accel = actor_data["accel"]
            accel_covariance = object_utils.generate_covariance(linear_variance, angular_variance)
            object.kinematics.acceleration_with_covariance.covariance = accel_covariance
            object.kinematics.is_stationary = False

            object.shape.type = 0 
            object.shape.footprint.points = []
            object.shape.dimensions.x = abs(actor_data["boundingbox"].x*2)
            object.shape.dimensions.y = abs(actor_data["boundingbox"].y*2)
            object.shape.dimensions.z = abs(actor_data["boundingbox"].z*2)
            output_msg.objects.append(object)

        self.itv_publisher.publish(output_msg)
        self.get_logger().info('Intervening track information')
    
    def detected_object(self):
        output_msg = DetectedObjects()
        output_msg.header = self.get_header()
        output_msg.header.frame_id = "base_link"
        object_utils = ObjectUtils()
        actors = self.get_actor_data()
        for actor_data in actors:
            object  = DetectedObject()
            object.existence_probability= 1.0
            """
            return a map function 
            """
            object_id = actor_data["type"]
            obj_classification = ObjectClassification()
            obj_classification.label = object_utils.map_object_type(object_id)
            obj_classification.probability = 1.0
            object.classification.append(obj_classification)

            object.kinematics.pose_with_covariance.pose = actor_data["obstacle_ros_pose"]
            object.kinematics.pose_with_covariance.pose.orientation.x = round(object.kinematics.pose_with_covariance.pose.orientation.x,9)
            object.kinematics.pose_with_covariance.pose.orientation.y = round(object.kinematics.pose_with_covariance.pose.orientation.y,9)
            object.kinematics.pose_with_covariance.pose.orientation.z = round(object.kinematics.pose_with_covariance.pose.orientation.z,9)
            object.kinematics.pose_with_covariance.pose.position.z += 1

            linear_variance = (0.0, 0.0, 0.0)
            angular_variance = (0.0, 0.0, 0.0)
            pose_covariance = object_utils.generate_covariance(linear_variance, angular_variance)
            object.kinematics.pose_with_covariance.covariance = pose_covariance

            object.kinematics.has_position_covariance = False
            object.kinematics.orientation_availability = 1

            object.kinematics.twist_with_covariance.twist = actor_data["twist"]
            object.kinematics.twist_with_covariance.twist.linear.x = 0.0
            object.kinematics.twist_with_covariance.twist.linear.y = 0.0
            object.kinematics.twist_with_covariance.twist.linear.z = 0.0
            object.kinematics.twist_with_covariance.twist.angular.x = 0.0
            object.kinematics.twist_with_covariance.twist.angular.y = 0.0
            object.kinematics.twist_with_covariance.twist.angular.z = 0.0
            twist_covariance = object_utils.generate_covariance(linear_variance, angular_variance)
            object.kinematics.twist_with_covariance.covariance = twist_covariance
            object.kinematics.has_twist = False
            object.kinematics.has_twist_covariance = False

            object.shape.type = 0 
            object.shape.footprint.points = []
            object.shape.dimensions.x = abs(actor_data["boundingbox"].x*2) 
            object.shape.dimensions.y = abs(actor_data["boundingbox"].y*2)
            object.shape.dimensions.z = abs(actor_data["boundingbox"].z*2)
            output_msg.objects.append(object)

        self.itv_publisher.publish(output_msg)
        self.get_logger().info('Intervening detection information')    

    def detected_roi(self):
        output_msg = DetectedObjectsWithFeature()
        output_msg.header = self.get_header()
        output_msg.header.frame_id = "camera0/camera_link"
        object_utils = ObjectUtils()
        actors = self.get_actor_data()
        for actor_data in actors:
            object  = DetectedObject()
            # TODO: Figure out why the existence_probability is 0.0
            object.existence_probability= 0.0
            """
            return a map function 
            """
            object_id = actor_data["type"]
            obj_classification = ObjectClassification()
            obj_classification.label = object_utils.map_object_type(object_id)
            obj_classification.probability = 1.0
            object.classification.append(obj_classification)

            object.kinematics.pose_with_covariance.pose = actor_data["obstacle_ros_pose"]
            object.kinematics.pose_with_covariance.pose.position.z += 1

            linear_variance = (0.0, 0.0, 0.0)
            angular_variance = (0.0, 0.0, 0.0)
            pose_covariance = object_utils.generate_covariance(linear_variance, angular_variance)
            object.kinematics.pose_with_covariance.covariance = pose_covariance

            object.kinematics.has_position_covariance = False
            object.kinematics.orientation_availability = 0

            object.kinematics.twist_with_covariance.twist = actor_data["twist"]
            # object.kinematics.twist_with_covariance.twist.linear.x = 0.0
            # object.kinematics.twist_with_covariance.twist.linear.y = 0.0
            # object.kinematics.twist_with_covariance.twist.linear.z = 0.0
            twist_covariance = object_utils.generate_covariance(linear_variance, angular_variance)
            object.kinematics.twist_with_covariance.covariance = twist_covariance
            object.kinematics.has_twist = False
            object.kinematics.has_twist_covariance = False

            object.shape.type = 0 
            object.shape.footprint.points = []
            object.shape.dimensions.x = 0.0
            object.shape.dimensions.y = 0.0
            object.shape.dimensions.z = 0.0

            feature_object = DetectedObjectWithFeature()
            feature_object.object = object
            feature_object.feature = object_utils.generate_roi_feature()
            output_msg.feature_objects.append(feature_object)

        self.itv_publisher.publish(output_msg)
        self.get_logger().info('Intervening roi information')  

    def detect_object_withfeature(self):
        output_msg = DetectedObjectsWithFeature()
        output_msg.header = self.get_header()
        output_msg.header.frame_id = "base_link"
        object_utils = ObjectUtils()
        actors = self.get_actor_data()
        for actor_data in actors:
            object  = DetectedObject()
            # TODO: Figure out why the existence_probability is 0.0
            object.existence_probability= 0.0
            """
            return a map function 
            """
            object_id = actor_data["type"]
            obj_classification = ObjectClassification()
            obj_classification.label = object_utils.map_object_type(object_id)
            obj_classification.probability = 1.0
            object.classification.append(obj_classification)

            object.kinematics.pose_with_covariance.pose = actor_data["obstacle_ros_pose"]
            object.kinematics.pose_with_covariance.pose.position.z += 1

            linear_variance = (0.0, 0.0, 0.0)
            angular_variance = (0.0, 0.0, 0.0)
            pose_covariance = object_utils.generate_covariance(linear_variance, angular_variance)
            object.kinematics.pose_with_covariance.covariance = pose_covariance

            object.kinematics.has_position_covariance = False
            object.kinematics.orientation_availability = 0

            object.kinematics.twist_with_covariance.twist = actor_data["twist"]
            twist_covariance = object_utils.generate_covariance(linear_variance, angular_variance)
            object.kinematics.twist_with_covariance.covariance = twist_covariance
            object.kinematics.has_twist = False
            object.kinematics.has_twist_covariance = False

            object.shape.type = 0 
            object.shape.footprint.points = []
            object.shape.dimensions.x = 0.0
            object.shape.dimensions.y = 0.0
            object.shape.dimensions.z = 0.0

            feature_object = DetectedObjectWithFeature()
            feature_object.object = object
            feature_object.feature = object_utils.generate_cluster_feature(self.get_header())
            output_msg.feature_objects.append(feature_object)

        self.itv_publisher.publish(output_msg)
        self.get_logger().info('Intervening feature information')

node_topics = [
    {
        "Node_label": "lidar_centerpoint",
        "publish_topic": "/perception/object_recognition/detection/centerpoint/objects",
        "msg_type":"DetectedObjects",
        "call_back":"detected_object"
    },
    {
        "Node_label": "detected_object_feature_remover",
        "publish_topic": "/perception/object_recognition/detection/clustering/camera_lidar_fusion/objects",
        "msg_type":"DetectedObjects",
        "call_back":"detected_object"
    },
    {
        "Node_label": "roi_cluster_fusion",
        "publish_topic": "/perception/object_recognition/detection/clustering/camera_lidar_fusion/clusters",
        "msg_type":"DetectedObjectsWithFeature",
        "call_back":"detect_object_withfeature"
    },
    {
        "Node_label": "fusion_shape_estimation",
        "publish_topic": "/perception/object_recognition/detection/clustering/camera_lidar_fusion/objects_with_feature",
        "msg_type":"DetectedObjectsWithFeature",
        "call_back":"detect_object_withfeature"
    },
    {
        "Node_label": "clustering_shape_estimation",
        "publish_topic": "/perception/object_recognition/detection/clustering/objects_with_feature",
        "msg_type":"DetectedObjectsWithFeature",
        "call_back":"detect_object_withfeature"
    },
    {
        "Node_label": "euclidean_cluster",
        "publish_topic": "/perception/object_recognition/detection/clustering/clusters",
        "msg_type":"DetectedObjectsWithFeature",
        "call_back":"detect_object_withfeature"
    },
    {
        "Node_label": "object_association_merger_0",
        "publish_topic": "/perception/object_recognition/detection/camera_lidar_fusion/objects",
        "msg_type":"DetectedObjects",
        "call_back":"detected_object"
    },
    {
        "Node_label": "object_association_merger_1",
        "publish_topic": "/perception/object_recognition/detection/objects",
        "msg_type":"DetectedObjects",
        "call_back":"detected_object"
    },
    {
        "Node_label":"obstacle_pointcloud_based_validator",
        "publish_topic": "/perception/object_recognition/detection/centerpoint/validation/objects",
        "msg_type":"DetectedObjects",
        "call_back":"detected_object"
    },
      {
        "Node_label":"detection_by_tracker",
        "publish_topic": "/perception/object_recognition/detection/detection_by_tracker/objects",
        "msg_type":"DetectedObjects",
        "call_back":"detected_object"
    },
    {
        "Node_label":"tensorrt_yolo",
        "publish_topic": "/perception/object_recognition/detection/rois0",
        "msg_type":"DetectedObjectsWithFeature",
        "call_back":"detect_object_withfeature"
 
    },
    {
        "Node_label":"multi_object_tracker",
        "publish_topic": "/perception/object_recognition/tracking/objects", 
        "msg_type":"TrackedObjects",
        "call_back":"tracked_object"
    }
] 
def get_node_info(node_labels):
    nodes_info = []
    for node_label in node_labels:
        for node in node_topics:
            if node["Node_label"] == node_label:
                nodes_info.append(node)
                break
    return nodes_info

def create_node(item):
    msg_type_class = {
        'DetectedObjects': DetectedObjects,
        'TrackedObjects': TrackedObjects,
        'DetectedObjectsWithFeature': DetectedObjectsWithFeature,
        # Add other message types here as needed
    }.get(item['msg_type'], None)

    if msg_type_class is None:
        raise ValueError(f"Message type '{item['msg_type']}' is not a valid/known message type.")
    
    callback = item['call_back'] 
    if callback is None:
        raise ValueError(f"Callback '{item['call_back']}' is not a valid/known callback.")
    
    node_name = f"interve_{item['Node_label']}"
    return GroundTruthListener(node_name,item['publish_topic'], callback, msg_type_class)

def do(targets,args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    nodes = []
    target_nodes = get_node_info(targets)
    for item in target_nodes:
        try:
            node = create_node(item)
            nodes.append(node)
            executor.add_node(node)
        except ValueError as e:
            print(e)
            continue
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process some targets.")
    parser.add_argument('--targets', nargs='+', help='List of targets to process')
    # targets =["euclidean_cluster"]
    targets = ['obstacle_pointcloud_based_validator']
    args = parser.parse_args()
    do(args.targets)