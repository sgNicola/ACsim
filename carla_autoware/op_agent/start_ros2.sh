#! /bin/bash -e
role_name=$1
map_name=$2
explore_mode=$3
route_topic=$4

echo "-------------------------------"
echo "Starting OpenPlanner .. "
echo $role_name
echo $map_name
echo $explore_mode
echo $route_topic
echo "-------------------------------"

source /home/anonymous/ACsim/autoware/install/setup.sh
# source /home/anonymous/autoware/install/setup.bash
ros2 launch /home/anonymous/ACsim/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml map_path:=/home/anonymous/ACsim/map_data/${map_name} vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit yolo_faulty_mode:=0 lidar_faulty_mode:=0 validation_faulty_mode:=0 shape_faulty_mode:=0 fusion_faulty_mode:=0 merger_faulty_mode:=0 dbt_faulty_mode:=0 tracker_faulty_mode:=1 yolo_timeLatencyDuration:=0 lidar_timeLatencyDuration:=0 shape_timeLatencyDuration:=0 fusion_timeLatencyDuration:=0 merger_timeLatencyDuration:=0 dbt_timeLatencyDuration:=0 tracker_timeLatencyDuration:=0

# ros2 launch /home/anonymous/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml map_path:=/home/anonymous/map_data/${map_name} vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

#$SHELL
