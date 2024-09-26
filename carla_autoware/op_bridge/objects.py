import math
import os
import subprocess
import signal
import threading
import time
import numpy
import numpy as np
import carla
from transforms3d.euler import euler2mat, quat2euler, euler2quat
import rclpy
from rclpy.clock import ClockType
# from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TwistWithCovariance, TwistStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, NavSatStatus, CameraInfo, Range, PointField, Imu
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header, String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from autoware_auto_vehicle_msgs.msg import ControlModeReport, GearReport, SteeringReport, TurnIndicatorsReport, HazardLightsReport, VelocityReport
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_perception_msgs.msg import TrackedObjects
from rclpy.node import Node
from carla_msg.msg import CarlaActorList, CarlaActor
"""
Base Classes to handle Actor objects
"""
import carla_common.transforms as trans

# from carla_ros_bridge.pseudo_actor import PseudoActor

from geometry_msgs.msg import TransformStamped  # pylint: disable=import-error

 
 
#         """
#         Function to provide the current ROS pose

#         :return: the ROS pose of this actor
#         :rtype: geometry_msgs.msg.Pose
 
#         Function to provide the current ROS pose

#         :return: the ROS pose of this actor
#         :rtype: geometry_msgs.msg.Pose
 
#         Function to provide the current ROS twist

#         :return: the ROS twist of this actor
#         :rtype: geometry_msgs.msg.Twist
 
 
#         Function to provide the current ROS accel

#         :return: the ROS twist of this actor
#         :rtype: geometry_msgs.msg.Twist

# class JsonInfo:
#     @classmethod
#     def frame_info(self, ego_vehicle,frame_id):
#             # frame_info ={"time_id":time_stamp,"frame_id":frame_id, "ego_vehicle":ego_info,"objects": objects}
#             return frame_info
    
#     @staticmethod
#     def save_file(data,timestamp, file_path):
#         """
#         Saves the data to a file at the specified file path.
#         """
#         try:
#             # record time
#             record_time = time.strftime("%Y_%m_%d-%H_%M_%S", time.localtime())
#             filename = f"{record_time}.json"
#             file =os.path.join(file_path,filename)  
#             with open(file, 'w') as file:
#                 json.dump(data, file, indent=4, ensure_ascii=False)
#             print(f"File saved successfully at: {file}")
#             return True
#         except Exception as e:
#             print(f"An error occurred while saving the file: {e}")
#             return False
  

class ActorPublisherNode(Node):
    def __init__(self):
        super().__init__('actor_publisher_node')
        self.actor_pub = self.create_publisher(CarlaActorList, '/carla_actors', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self._local_host = "localhost"
        self._port = 2000
        self.client = carla.Client(self._local_host, self._port)
        self.world = self.client.get_world()
        self.ego_vehicle = None
        self.role_name ="hero"
        settings = self.world.get_settings()
        self.world.apply_settings(settings)
        self.bb_save_radius = 50

    def timer_callback(self):
        # Get actor data from the CARLA simulator
        actor_data = self.get_actor_data()

        # Publish actor data
        actor_msg = self.create_actor_message(actor_data)
        self.actor_pub.publish(actor_msg)

    def get_actor_data(self):
        # Fetch actor data from the CARLA simulator
        objects =[]
        self._actors = self.world.get_actors().filter('*vehicle*') 
        # actor_type = '*vehicle*'
        for car in self._actors:
            if car.attributes['role_name'] == "hero":
                self.ego_vehicle = car
        ego_vehicle = self.ego_vehicle
        actor_current_ros_pose = trans.carla_transform_to_ros_pose(ego_vehicle.get_transform())
        actor_current_ros_twist_rotated = trans.carla_velocity_to_ros_twist(ego_vehicle.get_velocity(),
                                                                            ego_vehicle.get_angular_velocity())
        actor_current_ros_accel  = trans.carla_acceleration_to_ros_accel(ego_vehicle.get_acceleration())
        actor_extent = actor_extent = trans.carla_vector_to_ros_vector_rotated(ego_vehicle.bounding_box.extent,ego_vehicle.bounding_box.rotation)    
        carla_actor_id = ego_vehicle.id
        carla_actor_type = ego_vehicle.type_id
        dist =0
        object ={
            "type": "ego_vehicle",
            "id": int(carla_actor_id),
            "pose": actor_current_ros_pose,
            "twist": actor_current_ros_twist_rotated,
            "accel": actor_current_ros_accel,
            "boundingbox":  actor_extent,
            "distance": dist
        }
        objects.append(object)

        actors = self._actors.filter('*vehicle*')
        for carla_actor in actors:
            dist = carla_actor.get_transform().location.distance(ego_vehicle.get_transform().location)
            blx = carla_actor.bounding_box.location.x
            if dist < self.bb_save_radius and not math.isnan(blx):
                if carla_actor.id != ego_vehicle.id:
                    actor_current_ros_pose = trans.carla_transform_to_ros_pose(carla_actor.get_transform())
                    actor_current_ros_twist_rotated = trans.carla_velocity_to_ros_twist(carla_actor.get_velocity(),
                                                                                        carla_actor.get_angular_velocity())
                    actor_current_ros_accel  = trans.carla_acceleration_to_ros_accel(carla_actor.get_acceleration())
                    actor_extent = trans.carla_vector_to_ros_vector_rotated(carla_actor.bounding_box.extent,carla_actor.bounding_box.rotation)
                    carla_actor_id = carla_actor.id
                    carla_actor_type = carla_actor.type_id
                    object ={
                        "type":carla_actor_type,
                        "id": int(carla_actor_id),
                        "pose": actor_current_ros_pose,
                        "twist": actor_current_ros_twist_rotated,
                        "accel": actor_current_ros_accel,
                        "boundingbox":  actor_extent,
                        "distance": dist
                    }
                    objects.append(object)
        
          
        walker_actors = self._actors.filter("*walker*")
        for carla_actor in walker_actors:
            dist = carla_actor.get_transform().location.distance(ego_vehicle.get_transform().location)
            blx = carla_actor.bounding_box.location.x
            if dist < self.bb_save_radius and not math.isnan(blx):
                if carla_actor.id != ego_vehicle.id:
                    actor_current_ros_pose = trans.carla_transform_to_ros_pose(carla_actor.get_transform())
                    actor_current_ros_twist_rotated = trans.carla_velocity_to_ros_twist(carla_actor.get_velocity(),
                                                                                        carla_actor.get_angular_velocity())
                    actor_current_ros_accel  = trans.carla_acceleration_to_ros_accel(carla_actor.get_acceleration())
                    actor_extent = trans.carla_vector_to_ros_vector_rotated(carla_actor.bounding_box.extent,carla_actor.bounding_box.rotation)
                    carla_actor_id = carla_actor.id
                    carla_actor_type = carla_actor_type.type_id
                    object ={
                        "type":carla_actor_type,
                        "id": int(carla_actor_id),
                        "pose": actor_current_ros_pose,
                        "twist": actor_current_ros_twist_rotated,
                        "accel": actor_current_ros_accel,
                        "boundingbox":  actor_extent,
                        "distance": dist
                    }
                    objects.append(object)
        
        static_actors = self._actors.filter("*static*")
        for carla_actor in static_actors:
            dist = carla_actor.get_transform().location.distance(ego_vehicle.get_transform().location)
            blx = carla_actor.bounding_box.location.x
            if dist < self.bb_save_radius and not math.isnan(blx):
                if carla_actor.id != ego_vehicle.id:
                    actor_current_ros_pose = trans.carla_transform_to_ros_pose(carla_actor.get_transform())
                    actor_current_ros_twist_rotated = trans.carla_velocity_to_ros_twist(carla_actor.get_velocity(),
                                                                                        carla_actor.get_angular_velocity())
                    actor_current_ros_accel  = trans.carla_acceleration_to_ros_accel(carla_actor.get_acceleration())
                    actor_extent = trans.carla_vector_to_ros_vector_rotated(carla_actor.bounding_box.extent,carla_actor.bounding_box.rotation)
                    carla_actor_id = carla_actor.id
                    carla_actor_type = carla_actor_type.type_id
                    object ={
                        "type":carla_actor_type,
                        "id": int(carla_actor_id),
                        "pose": actor_current_ros_pose,
                        "twist": actor_current_ros_twist_rotated,
                        "accel": actor_current_ros_accel,
                        "boundingbox":  actor_extent,
                        "distance": dist
                    }
                    objects.append(object)
        
        return objects
    
    def create_actor_message(self, actor_data):
        # Create a CarlaActorList message to publish actor data
        actor_msg = CarlaActorList()
        actor_msg.header.stamp = self.get_clock().now().to_msg()
        actor_msg.header.frame_id = 'map'
        
        for actor_info in actor_data:
            actor = CarlaActor()
            actor.id = actor_info['id']
            actor.type = actor_info['type']
            actor.pose = actor_info["pose"]
            actor.twist = actor_info['twist']
            actor.accel = actor_info['accel']
            actor.boundingbox = actor_info['boundingbox']
            # Set other actor attributes as needed
            
            actor_msg.actors.append(actor)
        
        return actor_msg

def main(args=None):
    rclpy.init(args=args)
    actor_publisher_node = ActorPublisherNode()
 
    # Start the ROS 2 node
    rclpy.spin(actor_publisher_node)
    actor_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()        


