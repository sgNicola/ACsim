from geometry_msgs.msg import Pose  
class MessageSerialize:
    def pose_serialize(pose):
        pose_dict = {
            'position': {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z
            },
            'orientation': {
                'x': pose.orientation.x,
                'y': pose.orientation.y,
                'z': pose.orientation.z,
                'w': pose.orientation.w
            }
        }
        return pose_dict

    def twist_serialize(obj):
            twist_dict = {
                'linear': {
                    'x': obj.linear.x,
                    'y': obj.linear.y,
                    'z': obj.linear.z
                },
                'angular': {
                    'x': obj.angular.x,
                    'y': obj.angular.y,
                    'z': obj.angular.z
                }
            }
            return twist_dict

    def accel_serialize(obj):
            accel_dict = {
                'linear': {
                    'x': obj.linear.x,
                    'y': obj.linear.y,
                    'z': obj.linear.z
                },
                'angular': {
                    'x': obj.angular.x,
                    'y': obj.angular.y,
                    'z': obj.angular.z
                }
            }
            return accel_dict

    def boundingbox_serialize(obj):
        boundingbox_dict ={
            'x': obj.x,
            'y': obj.y,
            'z':obj.z
        }
        return boundingbox_dict
    
    def  geometry_polygon_serialize(polygon):
        polygon_dict = {
            "points": []
        }

        for point in polygon.points:
            point_dict = {
                "x": point.x,
                "y": point.y,
                "z": point.z
            }
            polygon_dict["points"].append(point_dict)
        return polygon_dict

    def sensor_roi_serialize(roi):

        roi_dict = {
            "x_offset": roi.x_offset,
            "y_offset": roi.y_offset,
            "width": roi.width,
            "height": roi.height,
            "do_rectify": roi.do_rectify
        }

        return (roi_dict)


    def pointcloud2_serialize(pointcloud):
        pointcloud_dict = {
            "header": {
                "stamp": pointcloud.header.stamp.sec,
                "frame_id": pointcloud.header.frame_id,
            },
            "height": pointcloud.height,
            "width": pointcloud.width,
            "fields": [],
            "is_bigendian": pointcloud.is_bigendian,
            "point_step": pointcloud.point_step,
            "row_step": pointcloud.row_step,
            "data": pointcloud.data.tolist(),
            "is_dense": pointcloud.is_dense,
        }

        for field in pointcloud.fields:
            field_dict = {
                "name": field.name,
                "offset": field.offset,
                "datatype": field.datatype,
                "count": field.count,
            }
            pointcloud_dict["fields"].append(field_dict)

        return pointcloud_dict
    
    def dict2pose(pose_dict):
       # Create a new Pose message
        ros_pose = Pose()
        # Set the position
        ros_pose.position.x = pose_dict['position']['x']
        ros_pose.position.y = pose_dict['position']['y']
        ros_pose.position.z = pose_dict['position']['z']

        # Set the orientation
        ros_pose.orientation.x = pose_dict['orientation']['x']
        ros_pose.orientation.y = pose_dict['orientation']['y']
        ros_pose.orientation.z = pose_dict['orientation']['z']
        ros_pose.orientation.w = pose_dict['orientation']['w']
        return ros_pose