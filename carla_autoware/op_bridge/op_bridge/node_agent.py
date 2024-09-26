#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
This module provides a ROS autonomous agent interface to control the ego vehicle via a ROS stack
"""
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
from rclpy.node import Node
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
from srunner.scenariomanager.carla_data_provider import *
from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from autoware_auto_vehicle_msgs.msg import ControlModeReport, GearReport, SteeringReport, TurnIndicatorsReport, HazardLightsReport, VelocityReport
from autoware_auto_control_msgs.msg import AckermannControlCommand
from carla_msg.msg import CarlaActorList, CarlaActor
import carla_common.transforms as trans
from autoware_auto_perception_msgs.msg import ObjectClassification
from autoware_auto_perception_msgs.msg import TrackedObjects, DetectedObject, DetectedObjectKinematics, DetectedObjects, TrackedObject, ObjectClassification 
from utilities import ObjectUtils

def get_entry_point():
    return 'Ros2Agent'


def get_distance(ego_vehicle, actor):
    distance_vector = actor.get_transform().location  - ego_vehicle.get_transform().location
    distance = math.sqrt(math.pow(distance_vector.x, 2) + math.pow(distance_vector.y, 2))
    return distance

class Ros2Agent(AutonomousAgent):

    """
    Base class for ROS-based stacks.

    Derive from it and implement the sensors() method.

    Please define TEAM_CODE_ROOT in your environment.
    The stack is started by executing $TEAM_CODE_ROOT/start.sh

    The sensor data is published on similar topics as with the carla-ros-bridge. You can find details about
    the utilized datatypes there.

    This agent expects a roscore to be running.
    """

    def setup(self, path_to_conf_file):
        """
        setup agent
        """
        self.track = Track.MAP
        self.agent_role_name = os.environ['AGENT_ROLE_NAME']
        self.bridge_mode = os.environ['OP_BRIDGE_MODE']
        self.topic_base = "/carla/{}".format(self.agent_role_name)
        self.stack_thread = None
        self.counter = 0
        self.open_drive_map_name = None
        self.open_drive_map_data = None       
                
        # get start_script from environment
        team_code_path = os.environ['TEAM_CODE_ROOT']
        if not team_code_path or not os.path.exists(team_code_path):
            raise IOError("Path '{}' defined by TEAM_CODE_ROOT invalid".format(team_code_path))
        self.start_script = "{}/start_ros2.sh".format(team_code_path)
        if not os.path.exists(self.start_script):
            raise IOError("File '{}' defined by TEAM_CODE_ROOT invalid".format(self.start_script))

        # initialize ros2 node
        rclpy.init(args=None)
        self.ros2_node = rclpy.create_node("node_agent")
        
        obj_clock = Clock()
        obj_clock.clock = Time(sec=0)
        self.clock_publisher.publish(obj_clock)
        
        self.timestamp = None
        self.speed = 0
        
        self.actor_pub = None
        self.itv_pub = None


        self.publisher_map = {}
        self.id_to_sensor_type_map = {}
        self.id_to_camera_info_map = {}
        self.cv_bridge = CvBridge()

        # self.objects_subscription = self.ros2_node.create_subscription(
        #     TrackedObjects,
        #     '/perception/object_recognition/tracking/objects',
        #     self.listener_callback,
        #     qos_profile=QoSProfile(depth=1))


        self.actor_pub = self.ros2_node.create_publisher(CarlaActorList, "/carla_actors",10)
        self.itv_publisher = self.ros2_node.create_publisher(TrackedObjects, '/test', 10)
        
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.ros2_node,))
        self.spin_thread.start()
  

    
    def get_actor_data(self):
        # Fetch actor data from the CARLA simulator
        world = CarlaDataProvider.get_world()
        bb_save_radius = 50
        vehicle_actors =  world.get_actors().filter('*vehicle*')
        walker_actors =  world.get_actors().filter('*walker*')
        static_actors = world.get_actors().filter('*static*')

        # actor_type = '*vehicle*'
         
        ego_vehicle = None
        other_actors = []
 
        for actor in vehicle_actors:
            if actor.attributes.get('role_name') == "hero":
                ego_vehicle = actor
            else:
                other_actors.append(actor)
        if ego_vehicle is None:
            raise ValueError("Ego vehicle (hero) not found among actors")
        ego_pose = trans.carla_transform_to_ros_pose(ego_vehicle.get_transform())
        ego_transform_matrix = trans.ros_pose_to_transform_matrix(ego_pose)
        for walker in walker_actors:
            other_actors.append(walker)
        for statics in static_actors:
            other_actors.append(statics)

        actor_data_within_radius = []
 
        for carla_actor in other_actors:
            distance = get_distance(ego_vehicle, carla_actor)
            if distance < bb_save_radius:
                bounding_box_location = actor.bounding_box.location
                if not math.isnan(bounding_box_location.x):
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
                        "distance": distance
                    }
                    actor_data_within_radius.append(object)
                # else:
        ego_current_ros_pose = trans.carla_transform_to_ros_pose(ego_vehicle.get_transform())
        ego_current_ros_twist_rotated = trans.carla_velocity_to_ros_twist(ego_vehicle.get_velocity(),
                                                                            ego_vehicle.get_angular_velocity())
        ego_current_ros_accel  = trans.carla_acceleration_to_ros_accel(ego_vehicle.get_acceleration())
        ego_extent = actor_extent = trans.carla_vector_to_ros_vector_rotated(ego_vehicle.bounding_box.extent,ego_vehicle.bounding_box.rotation)    
        carla_actor_id = ego_vehicle.id
        carla_actor_type = ego_vehicle.type_id
        ego_object ={
           "type": "ego_vehicle",
            "id": int(carla_actor_id),
            "pose": ego_current_ros_pose,
            "twist": ego_current_ros_twist_rotated,
            "accel": ego_current_ros_accel,
            "boundingbox":  ego_extent,
            "distance": distance
        }
                #     objects.append(object)
        return actor_data_within_radius, ego_object
        
    def get_header(self):
        """
        Returns ROS message header
        """
        header = Header()
        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)    

        #print('Sensor Time Stamp: ', header.stamp)    
        return header
    
    def publish_actor_message(self, actor_data):
        
        # Create a CarlaActorList message to publish actor data
        actor_msg = CarlaActorList()
        actor_msg.header = self.get_header()
        actor_msg.header.frame_id = 'map'
        if actor_data is not None:
            for actor_info in actor_data:
                actor = CarlaActor()
                actor.id = actor_info['id']
                actor.type = actor_info['type']
                actor.pose = actor_info["pose"]
                actor.twist = actor_info['twist']
                actor.accel = actor_info['accel']
                actor.boundingbox = actor_info['boundingbox']
                actor.distance = int(actor_info["distance"])
    
                # Set other actor attributes as needed
                
                actor_msg.actors.append(actor)
        
        self.actor_pub.publish(actor_msg)

    def publish_tracked_objects(self, actors):

        output_msg = TrackedObjects()
        output_msg.header = self.get_header()
        output_msg.header.frame_id = "map"
        object_utils = ObjectUtils()
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
            linear_variance = (0.0, 0.0, 0.0)
            angular_variance = (0.0, 0.0, 0.0)
            pose_covariance = object_utils.generate_twist_with_covariance_covariance(linear_variance, angular_variance)
            object.kinematics.pose_with_covariance.covariance = pose_covariance

            object.kinematics.orientation_availability = 1

            object.kinematics.twist_with_covariance.twist = actor_data["twist"]
            twist_covariance = object_utils.generate_twist_with_covariance_covariance(linear_variance, angular_variance)
            object.kinematics.twist_with_covariance.covariance = twist_covariance 

            object.kinematics.acceleration_with_covariance.accel = actor_data["accel"]
            accel_covariance = object_utils.generate_twist_with_covariance_covariance(linear_variance, angular_variance)
            object.kinematics.acceleration_with_covariance.covariance = accel_covariance
            object.kinematics.is_stationary = False

            object.shape.type = 0 
            object.shape.footprint.points = []
            object.shape.dimensions.x = abs(actor_data["boundingbox"].x*2)
            object.shape.dimensions.y = abs(actor_data["boundingbox"].y*2)
            object.shape.dimensions.z = abs(actor_data["boundingbox"].z*2)
            output_msg.objects.append(object)

        self.itv_publisher.publish(output_msg)

    def use_stepping_mode(self):  # pylint: disable=no-self-use
        """
        Overload this function to use stepping mode!
        """
        return False

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        """                
        self.timestamp = timestamp
        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        obj_clock = Clock()
        obj_clock.clock = Time(sec=seconds, nanosec=nanoseconds)

        if self.stack_process and self.stack_process.poll() is not None:
            raise RuntimeError("Stack exited with: {} {}".format(
                self.stack_process.returncode, self.stack_process.communicate()[0]))     
        actor_data, ego_data = self.get_actor_data()
        objects_data = actor_data.append(ego_data)
        self.publish_actor_message(objects_data)
        self.publish_tracked_objects(actor_data)

    def destroy(self):
        """
        Cleanup of all ROS publishers
        """        
        if self.stack_process and self.stack_process.poll() is None:
            # rospy.loginfo("Sending SIGTERM to stack...")
            os.killpg(os.getpgid(self.stack_process.pid), signal.SIGTERM)
            # rospy.loginfo("Waiting for termination of stack...")
            self.stack_process.wait()
            time.sleep(5)

 
