# Experiment config
camera_lidar_fusion based detection
config: rgb camera1, lidar 1,
detector:lidar_detection_model(centerpoint), camera_lidar_fusion(edu, yolox5)
Validator: pointcloud_map_filtered
Merger: 
tracker: multi_object_tracker

input topics:
/sensing/lidar/concatenated/pointcloud
/sensing/camera/camera0/image_rect_color
/map/vector_map
output topics:
/perception/*

topics to extract
1. /perception/object_recognition/detection/rois0
2. /perception/object_recognition/detection/clustering/clusters
3. /perception/object_recognition/detection/clustering/camera_lidar_fusion/objects
4. /perception/object_recognition/detection/centerpoint/validation/objects
5. /perception/object_recognition/detection/objects
6. /perception/object_recognition/tracking/objects
7. /perception/object_recognition/detection/detection_by_tracker/objects
8. /perception/obstacle_segmentation/pointcloud

Monitor 

# perceptionIdentify
Data process  
Data root: Data_root ="/home/anonymous/workspace/home/"  
Ros_dir ="rosbag"   
ObjectData_dir = "Data/ObjectData"
CarlaGT = "Data/CarlaGT"  

---------raw data--------------

rosbag:
    RosbagName

carlalog:
    logname

--------------------------------
ObjectData_dir
    RosbagName:
        Topic1: 
                .json
        Topic2:
                .json
    ......
CarlaGT:
    logname:
            .json
    

# Python Scikit-fuzzy
This package implements many useful tools for projects involving fuzzy logic, also known as grey logic. 
scikit-fuzzy (a.k.a. skfuzzy): Fuzzy logic toolbox for Python.