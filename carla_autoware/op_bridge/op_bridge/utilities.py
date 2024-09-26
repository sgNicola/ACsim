import uuid
class ObjectUtils(object):
    def __init__(self):
        self.UNKNOWN=['static.prop.streetbarrier',
        'static.prop.constructioncone',
        'static.prop.trafficcone01',
        'static.prop.trafficcone02',
        'static.prop.warningconstruction',
        'static.prop.warningaccident',
        'static.prop.foodcart',
        'static.prop.trafficwarning',
        ]
        self.CAR = ['vehicle.audi.a2',
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
        self.TRUCK = ['vehicle.carlamotors.carlacola',
        'vehicle.carlamotors.european_hgv',
        'vehicle.carlamotors.firetruck',
        'vehicle.tesla.cybertruck'
        ]
        self.BUS  = ['vehicle.mitsubishi.fusorosa']
        # motorcycle
        self.MOTORCYCLE = ['vehicle.yamaha.yzf',
        'vehicle.harley-davidson.low_rider',
        'vehicle.vespa.zx125',
        'vehicle.kawasaki.ninja']

        self.TRAILER=['vehicle.ford.ambulance',
        'vehicle.mercedes.sprinter',
        'vehicle.volkswagen.t2',
        'vehicle.volkswagen.t2_2021'
        ]
        # cyclist
        self.BICYCLE = ['vehicle.bh.crossbike',
        'vehicle.gazelle.omafiets',
        'vehicle.diamondback.century']
        self.PEDESTRIAN= ['walker.pedestrian.00'+f'{i:02d}' for i in range(1, 49)]
    
    def map_object_type(self, object_id):
            # Check the object_id against each category
        if object_id in self.UNKNOWN:
            return 0
        elif object_id in self.CAR:
            return 1
        elif object_id in self.TRUCK:
            return 2
        elif object_id in self.BUS:
            return 3
        elif object_id in self.MOTORCYCLE:
            return 4
        elif object_id in self.TRAILER:
            return 5
        elif object_id in self.BICYCLE:
            return 6
        elif object_id in self.PEDESTRIAN:
            return 7
        else:
            return 0  # For any object_id that doesn't match the known types

    # Mapping from type to number
    def type_to_number(vehicle_type):
        type_to_number_map = {
            "UNKNOWN": 0,
            "CAR": 1,
            "TRUCK": 2,
            "BUS": 3,
            "TRAILER": 4,
            "MOTORCYCLE": 5,
            "BICYCLE": 6,
            "PEDESTRIAN": 7
        }
        return type_to_number_map.get(vehicle_type, -1)

    def generate_twist_with_covariance_covariance(self, linear_var, angular_var):
        """
        Generate a 6x6 covariance matrix for a TwistWithCovariance message.
        
        Parameters:
        - linear_var: A tuple or list of variances for linear velocity (x, y, z).
        - angular_var: A tuple or list of variances for angular velocity (x, y, z).
        
        Returns:
        - A list of 36 float elements representing the covariance matrix.
        """
        assert len(linear_var) == 3 and all(isinstance(v, float) for v in linear_var), \
            "linear_var must be a sequence of three float values"
        assert len(angular_var) == 3 and all(isinstance(v, float) for v in angular_var), \
            "angular_var must be a sequence of three float values"
        
        # Initialize a 6x6 covariance matrix with all zeros
        covariance_matrix = [[0.0 for _ in range(6)] for _ in range(6)]
        
        # Set the variances for linear velocities
        covariance_matrix[0][0] = linear_var[0]  # Variance of linear velocity in X
        covariance_matrix[1][1] = linear_var[1]  # Variance of linear velocity in Y
        covariance_matrix[2][2] = linear_var[2]  # Variance of linear velocity in Z
        
        # Set the variances for angular velocities
        covariance_matrix[3][3] = angular_var[0]  # Variance of angular velocity around X
        covariance_matrix[4][4] = angular_var[1]  # Variance of angular velocity around Y
        covariance_matrix[5][5] = angular_var[2]  # Variance of angular velocity around Z
        
        # Flatten the matrix to a 1D array with 36 elements
        flat_covariance = [item for sublist in covariance_matrix for item in sublist]
        
        assert len(flat_covariance) == 36 and all(isinstance(v, float) for v in flat_covariance), \
            "The covariance field must be a sequence with length 36 and each value of type 'float'"
        
        return flat_covariance
        
    def generate_uuid_as_uint8_array():
        # Generate a random UUID
        random_uuid = uuid.uuid4()
        
        # Convert the UUID to a sequence of bytes
        # The UUID bytes are in big-endian order
        uuid_bytes = random_uuid.bytes
        
        # Convert bytes to a list of uint8 integers
        uuid_uint8_array = [b for b in uuid_bytes]
        
        return uuid_uint8_array