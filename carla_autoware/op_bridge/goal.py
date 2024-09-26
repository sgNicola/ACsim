import carla
from carla import VehicleLightState as vls

import argparse
import logging
from numpy import random
import math
import os
import subprocess
import signal
import threading
import time
import numpy
import numpy as np
import random
import carla
from transforms3d.euler import euler2mat, quat2euler, euler2quat
import rclpy
from rclpy.node import Node
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
from scipy.spatial.transform import Rotation
import carla_common.transforms as trans

class AutoPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
 
        self.publisher_ = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 1)
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

    def publish_goal(self, wp):
        """
        publish the global goal
        """
        msg = PoseStamped()
        msg.header = self.get_header()
        msg.header.frame_id = "map"
        
        # carla: pitch Y-axis, yaw Z-axis, roll X-axis
        # wp=self.spawnpoints # spawnpoints
        # wp.location.x = 50+random.randint(-10,10)
        # wp.location.y = 105.5
        # wp.location.z =1.2
        # wp.rotation.yaw =180
        # msg.pose.position.x = wp.location.x
        # msg.pose.position.y = -wp.location.y
        # msg.pose.position.z = wp.location.z
        # quaternion = euler2quat(0, 0, math.radians(wp.rotation.yaw))
        # msg.pose.orientation.w = quaternion[0]
        # msg.pose.orientation.x = quaternion[1]
        # msg.pose.orientation.y = quaternion[2]
        # msg.pose.orientation.z = quaternion[3]
        msg.pose = trans.carla_transform_to_ros_pose(wp)
        #  yaw_90 [0,0,-0.69,0.716] y_180[0,0,1,6.12] y_0[0,0,-0,1],y_270[0,0,0.71,0.69]
        print(msg.pose.orientation.w, msg.pose.orientation.z)

        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    # goal_point=[189.89,130.3,0]
    autopublisher = AutoPublisher()
    wp =carla.Transform(carla.Location(x=339.141052, y=100, z=0.0), carla.Rotation(pitch=0, yaw=-90, roll=0.000000))
    autopublisher.publish_goal(wp)
    rclpy.spin_once(autopublisher, timeout_sec=2)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    autopublisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()