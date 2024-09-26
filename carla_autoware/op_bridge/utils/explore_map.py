import sys
import carla
import numpy as np
# to generate a random spawning position or vehicles
import math
from os.path import isfile, join


class WorldHandler(object):

    def __init__(self):
        self._local_host = "localhost"
        self._port = 2000
        self._frame_rate = 20
        self.world = None
        self.client = None
        self.role_name ="hero"
        self.ego_vehicle = None
        self.bp_lib = None

    def load_world(self):
        self.client = carla.Client(self._local_host, self._port)
        self.client.set_timeout(20)    
 
        self.world = self.client.get_world()
        if self.world is not None:
            settings = self.world.get_settings()
            settings.fixed_delta_seconds = 1.0 / self._frame_rate
            settings.synchronous_mode = True
            self.world.apply_settings(settings)
        else:
            print("Can't Load CARLA .. make sure that Simulator is running !!")
        self.bp_lib = self.world.get_blueprint_library()  

        actors = self.world.get_actors().filter('*vehicle*')
        for car in actors:
            if car.attributes['role_name'] == self.role_name:
                self.ego_vehicle = car
        # spectator =self.world.get_spectator()
        # ego_trans = self.ego_vehicle.get_transform()
        # spectator.set_transform(carla.Transform(ego_trans.location + carla.Location(z=10),
        #                                             carla.Rotation(pitch=-90)))
 
    def generate_spawn_around(self, delta_x,delta_y ):
        # generate x, y around ego_vehicles
        ego_trans = self.ego_vehicle.get_transform()

        generate_location = carla.Transform(ego_trans.location+carla.Location(delta_x,delta_y,2),carla.Rotation(pitch= 0, yaw =90, roll =0))

        return generate_location

    def actor_bp(self, actor_type):
        # generate_bp
        #type ="vehicle.mercedes.sprinter"
        bp = self.bp_lib.find(actor_type)
        return bp
    
    def destroy_actors(self,actor_type):
        destroy_candidates = self.world.get_actors().filter(actor_type)
        for actor in destroy_candidates:
            print(actor.attributes)
            actor.destroy()
            # if actor.attributes["type"] == "vehicle.mercedes.sprinter":
        return None
    
    # def _create_object_from_actor(self, carla_actor, req=None):
    #     """
    #     create a object for a given carla actor
    #     Creates also the object for its parent, if not yet existing
    #     """
    #     # the transform relative to the map
    #     relative_transform = trans.carla_transform_to_ros_pose(carla_actor.get_transform())
    #     actor_transform_matrix = trans.ros_pose_to_transform_matrix(relative_transform)
    #     parent_transform_matrix = trans.ros_pose_to_transform_matrix(
    #         trans.carla_transform_to_ros_pose(self.ego_vehicle.get_transform()))
    #     relative_transform_matrix = np.matrix(
    #         parent_transform_matrix).getI() * np.matrix(actor_transform_matrix)
    #     object_transform = trans.transform_matrix_to_ros_pose(relative_transform_matrix)
    #     return relative_transform, object_transform
    
    def get_distance(ego_vehicle, actor):
        distance_vector = actor.get_transform().location  - ego_vehicle.get_transform().location
        distance = math.sqrt(math.pow(distance_vector.x, 2) + math.pow(distance_vector.y, 2))
        return distance

#     def get_actors():
#         world.get_actors().filter("vehicle*")
#         "*pedestrian*"
#         "static"

# class MinimalSubscriber(Node):

#     def __init__(self):
#         super().__init__('minimal_subscriber')
#         self.subscription = self.create_subscription(
#             TrackedObjects,
#             '/perception/object_recognition/tracking/objects',
#              callback=self.listener_callback,
#             qos_profile=QoSProfile(depth=1))
#         self.subscription  # prevent unused variable warning

#     def listener_callback(self, msg):
#         # number of detected objects
#         object= len(msg.objects)
#         actors =carla.world.get_actors()
#         self.get_logger().info('I heard: "%s at %s Time"' % (len(msg.objects),msg.header.stamp.sec))
#         #self.get_logger().info('I heard: "%s"' % msg.headers)
 
    
# def monitor(args=None):
#     rclpy.init(args=args)

#     minimal_subscriber = MinimalSubscriber()

#     rclpy.spin(minimal_subscriber)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_subscriber.destroy_node()
#     rclpy.shutdown()

def convert_transform_to_position(transform):
    """
    Convert an OpenScenario position into a CARLA transform

    Not supported: RoutePosition
    """
    position =[]
    x = transform.location.x
    y = transform.location.y
    z = transform.location.z
    h = math.radians(transform.rotation.yaw)
    p = math.radians(transform.rotation.pitch)
    r = math.radians(transform.rotation.roll)
    # y = y * (-1.0)
    h = h * (-1.0)
    position =[x,y,z,h,p,r]
    return position
# """
# Summary of useful helper functions for scenarios
# """
# def get_distance_between_actors(current, target, distance_type="euclidianDistance", freespace=False,
#                                 global_planner=None):
#     return None

if __name__=="__main__":
    world_handler = WorldHandler()
    world_handler.load_world()   
    client = world_handler.client
    world = world_handler.world
    tranform  = world_handler.ego_vehicle.get_transform()
    carla_map = world.get_map()
    print("ego_transform", tranform)
    relative_waypoint = carla_map.get_waypoint(tranform.location)
    road_id, ref_lane_id, ref_s = relative_waypoint.road_id, relative_waypoint.lane_id, relative_waypoint.s
    waypoint =world.get_map().get_waypoint_xodr(road_id, ref_lane_id, ref_s)
    print("waypoint", waypoint)
    worldposition = convert_transform_to_position(tranform)
    print(road_id, ref_lane_id, ref_s)
    print("worldposition", worldposition)