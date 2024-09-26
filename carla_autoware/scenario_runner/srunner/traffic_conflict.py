#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

 

from __future__ import print_function

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter, KeepVelocity, ChangeAutoPilot, WaypointFollower, StopVehicle)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToVehicle, InTriggerDistanceToLocation
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenarios.basic_scenario import BasicScenario



class TrafficConflict(BasicScenario):
    """
    This class holds everything required for a scenario in which another vehicle runs a red light
    in front of the ego, forcing it to react. This vehicles are 'special' ones such as police cars,
    ambulances or firetrucks.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=180):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._source_dist = 30
        self._sink_dist = 20
        self._direction = None
        self.timeout = timeout
 
        self._min_trigger_dist = 9.0  # Min distance to the collision location that triggers the adversary [m]
        self._speed_duration_ratio = 2.0
        self._speed_distance_ratio = 1.5

        # Get the CDP seed or at routes, all copies of the scenario will have the same configuration
        self._rng = CarlaDataProvider.get_random_seed()

        super(TrafficConflict, self).__init__("TrafficConflict",
                                                             ego_vehicles,
                                                             config,
                                                             world,
                                                             debug_mode,
                                                             criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        super()._initialize_actors(config)
        blueprint_library = self.world.get_blueprint_library()
        
        # Define spawn points for non-ego actors
        spawn_points = [carla.Transform(carla.Location(x=-75, y=129.24330, z=1), carla.Rotation(yaw=160)),
                        carla.Transform(carla.Location(x=-71, y=131.243301, z=1), carla.Rotation(yaw=-210))]
        
        vehicle1 = self.world.spawn_actor(blueprint_library.find('vehicle.volkswagen.t2_2021'), spawn_points[0])
        vehicle2 = self.world.spawn_actor(blueprint_library.find('vehicle.volkswagen.t2'), spawn_points[1])
        
        self.other_actors.extend([vehicle1, vehicle2])

    def _create_behavior(self):
        ego_vehicle = self.ego_vehicles[0]
        vehicle1, vehicle2 = self.other_actors[:2]

        # Both vehicles approach the intersection
        approach = py_trees.composites.Parallel("Approaching Intersection", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        approach.add_child(WaypointFollower(vehicle1, 30))
        approach.add_child(WaypointFollower(vehicle2, 30))

        # Collision trigger
        collision_trigger = InTriggerDistanceToVehicle(vehicle1, vehicle2, distance=5)
        
        # Stop vehicles upon collision trigger
        stop = py_trees.composites.Parallel("Stop Vehicles", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        stop.add_child(StopVehicle(vehicle1, 0))
        stop.add_child(StopVehicle(vehicle2, 0))

        # Ego vehicle maneuvers to avoid the collision
        lane_change = ChangeAutoPilot(ego_vehicle, True)

        # Main behavior sequence
        root = py_trees.composites.Sequence("RootSequence")
        # root.add_child(approach)
        root.add_child(collision_trigger)
        root.add_child(stop)
        root.add_child(lane_change)

        return root
 

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        """
        Remove all actors and traffic lights upon deletion
        """
        self.remove_all_actors()