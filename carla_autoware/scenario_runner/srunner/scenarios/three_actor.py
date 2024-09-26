import random

import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower,
                                                                      Idle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               DriveDistance,
                                                                               StandStill)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance

from srunner.tools.background_manager import Scenario2Manager

class ThreeActor(BasicScenario):

    """
    This class holds a scenario similar to FollowLeadingVehicle
    but there is an obstacle in front of the leading vehicle

    This is a single ego vehicle scenario
    """

    timeout = 200          # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True):
        """
        Setup all relevant parameters and create scenario
        """
        self._map = CarlaDataProvider.get_map()
        self._first_actor_location = 30
        self._second_actor_location = 20 
        self._third_actor_location = 10

        self._first_actor_speed = 10
        self._second_actor_speed = 1.5
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 20
        self._first_actor_transform = None
        self._second_actor_transform = None

        super(ThreeActor, self).__init__("ThreeActor",
                                                               ego_vehicles,
                                                               config,
                                                               world,
                                                               debug_mode,
                                                               criteria_enable=criteria_enable)
        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        first_actor_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_actor_location)
        second_actor_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._second_actor_location)
        third_actor_waypoint,_ = get_waypoint_in_distance(self._reference_waypoint, self._third_actor_location)
 
        self._first_actor_transform = carla.Transform(
            carla.Location(first_actor_waypoint.transform.location.x + self.config.X1,
                           first_actor_waypoint.transform.location.y + self.config.Y1,
                           first_actor_waypoint.transform.location.z),
            carla.Rotation(first_actor_waypoint.transform.rotation.pitch, first_actor_waypoint.transform.rotation.yaw + self.config.YAW1,
                           first_actor_waypoint.transform.rotation.roll))
 
 
        self._second_actor_transform = carla.Transform(
            carla.Location(second_actor_waypoint.transform.location.x + self.config.X2,
                           second_actor_waypoint.transform.location.y + self.config.Y2,
                           second_actor_waypoint.transform.location.z),
            carla.Rotation(second_actor_waypoint.transform.rotation.pitch, second_actor_waypoint.transform.rotation.yaw + self.config.YAW2,
                           second_actor_waypoint.transform.rotation.roll))

        self._third_actor_transform = carla.Transform(
            carla.Location(third_actor_waypoint.transform.location.x + self.config.X3,
                           third_actor_waypoint.transform.location.y + self.config.Y3,
                           third_actor_waypoint.transform.location.z),
            carla.Rotation(third_actor_waypoint.transform.rotation.pitch, third_actor_waypoint.transform.rotation.yaw + self.config.YAW3,
                           third_actor_waypoint.transform.rotation.roll))


        car_types = self.config.car_types + self.config.buses_types + self.config.trucks_types
        motorcycle_type = self.config.motorcycle_types+self.config.cyclist_types

  

        try:
            first_actor = CarlaDataProvider.request_new_actor(
                random.choice(car_types), self._first_actor_transform
            )
        except Exception as e:
            # Handle the exception here, such as logging an error message or taking alternative actions
            print(f"Failed to create first actor: {e}")
            self._first_actor_transform = first_actor_waypoint.transform
            first_actor = CarlaDataProvider.request_new_actor(
                random.choice(car_types), self._first_actor_transform
            )

        try:
            second_actor = CarlaDataProvider.request_new_actor(
                random.choice(motorcycle_type), self._second_actor_transform
            )
        except Exception as e:
            print(f"Failed to create second actor: {e}")
            self._second_actor_transform =second_actor_waypoint.transform  
            second_actor = CarlaDataProvider.request_new_actor(
                random.choice(motorcycle_type), self._second_actor_transform
                )

        try:
            third_actor = CarlaDataProvider.request_new_actor(
                random.choice(motorcycle_type), self._third_actor_transform
            )
        except Exception as e:
            print(f"Failed to create third actor: {e}")
            self._third_actor_transform = third_actor_waypoint.transform
            third_actor = CarlaDataProvider.request_new_actor(
                random.choice(motorcycle_type), self._third_actor_transform
            )

        first_actor.set_simulate_physics(enabled=False)
        second_actor.set_simulate_physics(enabled=False)
        third_actor.set_simulate_physics(enabled=False)
        self.other_actors.append(first_actor)
        self.other_actors.append(second_actor)
        self.other_actors.append(third_actor)

        #  weather param
    def _initialize_environment(self, world):
        WEATHER = random.choice(self.config.weather_types)
        world.set_weather(WEATHER)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario.
        """
        # end condition
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=0,
                                                        name="FinalDistance")
        endcondition_part2 = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(ActorTransformSetter(self.other_actors[0], self._first_actor_transform))
        sequence.add_child(ActorTransformSetter(self.other_actors[1], self._second_actor_transform))
        sequence.add_child(ActorTransformSetter(self.other_actors[2], self._third_actor_transform))

        sequence.add_child(endcondition)

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
