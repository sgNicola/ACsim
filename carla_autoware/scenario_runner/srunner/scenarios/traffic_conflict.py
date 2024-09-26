import py_trees
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import *
from srunner.scenariomanager.scenarioatomics.atomic_criteria import *
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import *
from srunner.scenarios.basic_scenario import BasicScenario

class TrafficConflict(BasicScenario):

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=800):
        """
        Setup all relevant parameters and create scenario
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._ego_vehicle_drive_distance = 150  # Distance for the ego vehicle to drive before ending scenario
        self._collision_location = 50  # Location of the collision in the junction
        self._ego_vehicle_speed = 30 / 3.6  # Ego vehicle speed in m/s
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self.timeout = timeout

        super(TrafficConflict, self).__init__("TrafficConflict",
                                              ego_vehicles,
                                              config,
                                              world,
                                              debug_mode,
                                              criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        spawn_points = [carla.Transform(carla.Location(x=-75, y=129.24330, z=1), carla.Rotation(yaw=160)),
                        carla.Transform(carla.Location(x=-71, y=131.243301, z=1), carla.Rotation(yaw=-210))]

        front_vehicle_transform = spawn_points[0]
        left_vehicle_transform = spawn_points[1]

        front_vehicle = CarlaDataProvider.request_new_actor('vehicle.volkswagen.t2_2021', front_vehicle_transform)
        left_vehicle = CarlaDataProvider.request_new_actor('vehicle.volkswagen.t2', left_vehicle_transform)

        self.other_actors.append(front_vehicle)
        self.other_actors.append(left_vehicle)

    def _create_behavior(self):
        """
        The scenario involves the ego vehicle entering a junction where two other vehicles collide,
        forcing the ego vehicle to change lanes to avoid the blocked area.
        """
        # Start condition: Ego vehicle driving towards the junction
        sequence = py_trees.composites.Sequence("Scenario behavior")
        
        # Set transform for the actors
        sequence.add_child(ActorTransformSetter(self.other_actors[0], self.other_actors[0].get_transform()))
        sequence.add_child(ActorTransformSetter(self.other_actors[1], self.other_actors[1].get_transform()))

        # Trigger collision between other actors
        collision_trigger = py_trees.composites.Parallel("Collision trigger", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        collision_trigger.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0], self.other_actors[0].get_transform().location, 10))
        collision_trigger.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0], self.other_actors[1].get_transform().location, 10))

        collision_behavior = py_trees.composites.Sequence("Collision behavior")
        collision_behavior.add_child(ActorDestroy(self.other_actors[0]))
        collision_behavior.add_child(ActorDestroy(self.other_actors[1]))
        
        collision_trigger.add_child(collision_behavior)
        sequence.add_child(collision_trigger)

        # Ego vehicle has to change lane and drive a certain distance to end scenario
        ego_drive_distance = DriveDistance(self.ego_vehicles[0], self._ego_vehicle_drive_distance)
        sequence.add_child(ego_drive_distance)

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
        self.remove_all_actors()