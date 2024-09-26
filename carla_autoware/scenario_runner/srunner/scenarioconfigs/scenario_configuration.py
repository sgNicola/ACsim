#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the key configuration parameters for an XML-based scenario
"""

import carla


class ActorConfigurationData(object):

    """
    This is a configuration base class to hold model and transform attributes
    """

    def __init__(self, model, transform, rolename='other', speed=0, autopilot=False,
                 random=False, color=None, category="car", args=None):
        self.model = model
        self.rolename = rolename
        self.transform = transform
        self.speed = speed
        self.autopilot = autopilot
        self.random_location = random
        self.color = color
        self.category = category
        self.args = args

    @staticmethod
    def parse_from_node(node, rolename):
        """
        static method to initialize an ActorConfigurationData from a given ET tree
        """

        model = node.attrib.get('model', 'vehicle.*')

        pos_x = float(node.attrib.get('x', 0))
        pos_y = float(node.attrib.get('y', 0))
        pos_z = float(node.attrib.get('z', 0))
        yaw = float(node.attrib.get('yaw', 0))

        transform = carla.Transform(carla.Location(x=pos_x, y=pos_y, z=pos_z), carla.Rotation(yaw=yaw))

        rolename = node.attrib.get('rolename', rolename)

        speed = node.attrib.get('speed', 0)

        autopilot = False
        if 'autopilot' in node.keys():
            autopilot = True

        random_location = False
        if 'random_location' in node.keys():
            random_location = True

        color = node.attrib.get('color', None)

        return ActorConfigurationData(model, transform, rolename, speed, autopilot, random_location, color)


class ScenarioConfiguration(object):

    """
    This class provides a basic scenario configuration incl.:
    - configurations for all actors
    - town, where the scenario should be executed
    - name of the scenario (e.g. ControlLoss_1)
    - type is the class of scenario (e.g. ControlLoss)
    """

    trigger_points = []
    ego_vehicles = []
    other_actors = []
    town = None
    name = None
    type = None
    route = None
    agent = None
    weather = carla.WeatherParameters()
    friction = None
    subtype = None
    route_var_name = None

    X1 = 0
    Y1 = -1 #[left:-2.5,2.5 right]
    X2 = 0
    Y2 = 0
    X3 = 1
    Y3 = 1
    YAW1 = 0
    YAW2 = 0
    YAW3 = 0
    road_obstacle_type = 'static.prop.trafficcone01'
    sparse_obstacle_type = 'static.prop.trafficwarning'
    # car_type = 'vehicle.tesla.model3'
    car_type =  'vehicle.mercedes.coupe'
    truck_type = 'vehicle.carlamotors.carlacola'
    hugetruck_type = "vehicle.carlamotors.firetruck"
    buses_type = 'vehicle.mitsubishi.fusorosa'
    motorcycle_type = 'vehicle.yamaha.yzf'
    vans_type ='vehicle.ford.ambulance'
    # weather_type =carla.WeatherParameters.ClearSunset
    weather_type = carla.WeatherParameters.MidRainyNoon
    pedestrian_type = 'walker.pedestrian.0001'
    vehicle_color = '(255, 255, 255)'
    cyclist_type = 'vehicle.yamaha.yzf'

    road_obstacle_types=['static.prop.streetbarrier',
    'static.prop.constructioncone',
    'static.prop.trafficcone01',
    'static.prop.trafficcone02',
    'static.prop.warningconstruction',
    'static.prop.warningaccident',
    ]
    # 
    sparse_obstacle_types =['static.prop.foodcart',
                            'static.prop.trafficwarning',
                            ]
    car_types = ['vehicle.audi.a2',
    'vehicle.audi.tt',
    'vehicle.mercedes-benz.coupe',
    'vehicle.bmw.grandtourer',
    'vehicle.audi.etron',
    'vehicle.nissan.micra',
    'vehicle.lincoln.mkz_2017',
    'vehicle.dodge.charger_police',
    'vehicle.dodge.charger_police_2020',
    'vehicle.ford.crown',
    'vehicle.tesla.model3',
    'vehicle.toyota.prius',
    'vehicle.seat.leon',
    'vehicle.nissan.patrol',
    'vehicle.mini.cooperst',
    'vehicle.jeep.wrangler_rubicon',
    'vehicle.ford.mustang',
    'vehicle.volkswagen.t2',
    'vehicle.chevrolet.impala',
    'vehicle.dodge.charger_2020',
    'vehicle.citroen.c3']
    truck_types = ['vehicle.carlamotors.carlacola',
    # 'vehicle.carlamotors.european_hgv',
    'vehicle.carlamotors.firetruck',
    'vehicle.tesla.cybertruck'
    ]
    hugetruck_types = [ 
                       "vehicle.carlamotors.firetruck",
                       'vehicle.mitsubishi.fusorosa']
    # Those car have opening doors
    open_door_cars = ["vehicle.dodge.charger_2020",
                      "vehicle.dodge.charger_police",
                      "vehicle.dodge.charger_police_2020",
                      "vehicle.ford.crown",
                      "vehicle.lincoln.mkz_2020",
                      "vehicle.mercedes.coupe_2020",
                      "vehicle.mini.cooper_s_2021",
                      "vehicle.nissan.patrol_2021",
                      "vehicle.carlamotors.firetruck",
                      "vehicle.ford.ambulance",
                      "vehicle.mercedes.sprinter",
                      "vehicle.volkswagen.t2_2021",
                      ]


    buses_types = ['vehicle.mitsubishi.fusorosa']
    # motorcycle
    motorcycle_types = ['vehicle.yamaha.yzf',
    'vehicle.harley-davidson.low_rider',
    'vehicle.vespa.zx125',
    'vehicle.kawasaki.ninja']

    vans_types =['vehicle.ford.ambulance',
    'vehicle.mercedes.sprinter',
    'vehicle.volkswagen.t2',
    'vehicle.volkswagen.t2_2021'
    ]
    # cyclist
    cyclist_types = ['vehicle.bh.crossbike',
    'vehicle.gazelle.omafiets',
    'vehicle.diamondback.century']
    # walker modifiable attributes: speed: float
    pedestrian_types = ['walker.pedestrian.00'+f'{i:02d}' for i in range(1, 49)]
    
    vehicle_types = car_types + truck_types +  motorcycle_types +  cyclist_types + buses_types
    # map to the autoware label
    # black, white, gray, silver, blue, red, brown, gold, green, tan, orange
    vehicle_colors = ['(0, 0, 0)',
    '(255, 255, 255)',
    '(220, 220, 220)',
    '(192, 192, 192)',
    '(0, 0, 255)',
    '(255, 0, 0)',
    '(165,42,42)',
    '(255,223,0)',
    '(0,128,0)',
    '(210,180,140)',
    '(255,165,0)']

    weather_types = [
        carla.WeatherParameters.ClearNoon,
        carla.WeatherParameters.ClearSunset,

        carla.WeatherParameters.CloudyNoon,
        carla.WeatherParameters.CloudySunset,

        carla.WeatherParameters.WetNoon,
        carla.WeatherParameters.WetSunset,

        carla.WeatherParameters.MidRainyNoon,
        carla.WeatherParameters.MidRainSunset,

        carla.WeatherParameters.WetCloudyNoon,
        carla.WeatherParameters.WetCloudySunset,

        carla.WeatherParameters.HardRainNoon,
        carla.WeatherParameters.HardRainSunset,

        carla.WeatherParameters.SoftRainNoon,
        carla.WeatherParameters.SoftRainSunset,
        # ClearNight
        carla.WeatherParameters(15.0, 0.0, 0.0, 0.35, 0.0, -90.0, 0.0, 0.0, 0.0),

        # CloudyNight
        carla.WeatherParameters(80.0, 0.0, 0.0, 0.35, 0.0, -90.0, 0.0, 0.0, 0.0),
        # WetNight
        carla.WeatherParameters(20.0, 0.0, 50.0, 0.35, 0.0, -90.0, 0.0, 0.0, 0.0),
        # MidRainNight
        carla.WeatherParameters(90.0, 0.0, 50.0, 0.35, 0.0, -90.0, 0.0, 0.0, 0.0),
        # WetCloudyNight
        carla.WeatherParameters(80.0, 30.0, 50.0, 0.40, 0.0, -90.0, 0.0, 0.0, 0.0),
        # HardRainNight
        carla.WeatherParameters(80.0, 60.0, 100.0, 1.00, 0.0, -90.0, 0.0, 0.0, 0.0),
        # SoftRainNight
        carla.WeatherParameters(90.0, 15.0, 50.0, 0.35, 0.0, -90.0, 0.0, 0.0, 0.0)]