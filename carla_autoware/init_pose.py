# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================

import glob
import os
import sys
from numpy import random
import math
import numpy as np
import random
import carla
from transforms3d.euler import euler2mat, quat2euler, euler2quat
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from builtin_interfaces.msg import Time
from std_msgs.msg import Header, String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from scipy.spatial.transform import Rotation

class AutoPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
 
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.timestamp = 0
        
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

    def publish_goal(self,wp):
        """
        publish the global goal
        """
        msg =PoseWithCovarianceStamped()
        msg.header = self.get_header()
        msg.header.frame_id = "map"
        # yaw [0,90, 180, 270]
        #           90
        #           |
        #   180 <----x-----> 0
        #           |
        #           270
        msg.pose.pose.position.x = wp.location.x
        msg.pose.pose.position.y = -wp.location.y
        msg.pose.pose.position.z = wp.location.z
        quaternion = euler2quat(0, 0, -math.radians(wp.rotation.yaw))
        msg.pose.pose.orientation.w = quaternion[0]
        msg.pose.pose.orientation.x = quaternion[1]
        msg.pose.pose.orientation.y = quaternion[2]
        msg.pose.pose.orientation.z = quaternion[3]
        msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.25, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]
        #  yaw_90 [0,0,-0.69,0.716] y_180[0,0,1,6.12] y_0[0,0,-0,1],y_270[0,0,0.71,0.69]
        print(msg.pose.pose.orientation.w)

        self.publisher_.publish(msg)


def get_init_pose():
    # world = CarlaDataProvider.get_world()
    client = carla.Client("localhost",2000)
    frame_rate = 20.0 
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 1.0 / frame_rate
    world.apply_settings(settings)
    vehicle_actors = world.get_actors().filter('vehicle.*')
    for car in vehicle_actors:
        if car.attributes['role_name'] == "hero":
            ego_vehicle = car
    init_pose = ego_vehicle.get_transform()
    return init_pose

def main(args=None):
    rclpy.init(args=args)
    # goal_point=[189.89,130.3,0]
    autopublisher = AutoPublisher()
    wp= get_init_pose()
    autopublisher.publish_goal(wp)
    rclpy.spin_once(autopublisher, timeout_sec=2)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    autopublisher.destroy_node()
    rclpy.shutdown()
