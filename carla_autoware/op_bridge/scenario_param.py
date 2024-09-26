import math

import carla


# x	double	1..1	XSDattribute	The x coordinate value.
# y	double	1..1	XSDattribute	The y coordinate value.
# z	double	0..1	XSDattribute	The z coordinate value.
# h	double	0..1	XSDattribute	The heading angle of the object, defining a mathematically positive rotation about the z-axis (see ISO 8855:2011).
# p	double	0..1	XSDattribute	The pitch angle of the object, defining a mathematically positive rotation about the y-axis (see ISO 8855:2011).
# r	double	0..1	XSDattribute	The roll angle of the object, defining a mathematically positive rotation about the x-axis (see ISO 8855:2011).

def carla_point(pose):
    "x,y,z,pitch, yaw,roll"
    "(pitch, yaw, roll), which corresponds to (Y-rotation,Z-rotation,X-rotation)  (p,h,r)"
    [175.4,195.14,1.2,0,180,0]
    return carla.Transform(carla.Location(x=pose[0], y=pose[1], z=pose[2]), 
              carla.Rotation(pitch=pose[3] , yaw=pose[4] , roll=pose[5]))

def convert_transform_to_position(transform, actor_list=None):
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
    h = h * (-1.0)
    position =[x,y,z,h,p,r]
    return position

if __name__=="__main__":
    tranform = carla.Transform(carla.Location(x=221.569366, y=133.141312, z=0.036132), carla.Rotation(pitch=0, yaw=-90, roll=0.0))
    worldposition = convert_transform_to_position(tranform)
    print(worldposition)
