import subprocess
import os
import time
import signal
import re
import yaml
import init_pose
import shutil
import sys
from perceptionIdentify.fusion_launch import modify_and_save_xml
from perceptionIdentify.process_rosbag import process_rosbag_file
from param import Rosbag_dir,setup_path,carla_path, fusion_launch, fusion_launch_path,mutation_runner,agent,fusion_intervene,ObjectData_dir
from perceptionIdentify.parse_fusion import record_failure, record_intervene
import param

def generate_bag_record_command(topics,bag_name):
    cmd = ['ros2', 'bag', 'record','-o', bag_name]
    cmd.extend(topics)
    return cmd

def run_carla_scenario_agent(experiment_id, bag_name,scenario,params,targets=None):
    os.system("pkill -f 'ros'")
    os.system("pkill -f 'CarlaUE4'")
    os.system("pkill -f 'autoware'")
    time.sleep(2)
    record_path =os.path.join(Rosbag_dir,experiment_id)
    if not os.path.exists(record_path):
        os.makedirs(record_path)
    setup_shell = setup_path
    carla =  carla_path
    old_bag = os.path.join(record_path,bag_name)
    if os.path.exists(old_bag):
        shutil.rmtree(old_bag)
    input_file_path =  fusion_launch
    output_file_path =  fusion_launch_path
    if os.path.exists(output_file_path):
        os.remove(output_file_path)
    if targets is not None:
        modify_and_save_xml(input_file_path, output_file_path,targets) 
    else:
        shutil.copyfile(input_file_path, output_file_path)
    print("-------------------------------") 
    # Start Carla server
    carla_command = f"echo 'START CARLA 0.9.14'; {carla} -quality-level=Low"
    carla_process=subprocess.Popen(carla_command, shell=True,executable="/bin/bash")
    time.sleep(10)
    # Load scenario
    scenario_command = f"python3 {mutation_runner} --scenario {scenario} --reloadWorld"
    for key, value in params.items():
        scenario_command += f" --{key}={value}"
    scenario_command += " --sync"
    print(scenario_command)
    scenario_process=subprocess.Popen(scenario_command, shell=True)
    time.sleep(6)
    # Run Autoware agent
    autoware_command = f"echo 'SOURCE AUTOWARE'; source {setup_shell}; {agent}"
    autoware_process =subprocess.Popen(autoware_command, shell=True, executable="/bin/bash")
    time.sleep(45)
    init_pose.main()
    time.sleep(3)
    if targets is not None:
        targets_str = ' '.join(targets)
        intervene_command = f'gnome-terminal -- bash -c "source {setup_shell}; python3 {fusion_intervene} --targets {targets_str}"'
        intervene_process =subprocess.Popen(intervene_command, shell=True, executable="/bin/bash")
    with open("config.yaml", encoding='UTF-8') as yaml_file:
        topics = yaml.safe_load(yaml_file)
    time.sleep(4)
    bag_record_cmd = generate_bag_record_command(topics["record_topics"],bag_name)
    bag_record_cmd_str = ' '.join(bag_record_cmd)
    terminal_cmd = f'gnome-terminal -- bash -c "cd {record_path}; source {setup_shell}; {bag_record_cmd_str}"'
    record_process = subprocess.Popen(terminal_cmd, shell=True,executable="/bin/bash")
    time.sleep(15)
    record_process.send_signal(signal.SIGINT)
    record_process.wait()
    os.system("pkill -SIGINT -f 'record'")
    os.system("pkill -SIGINT -f 'oracle_listener'")
    time.sleep(3)
    scenario_process.send_signal(signal.SIGINT)
    time.sleep(1)
    os.system("pkill -f 'run_srunner_agent_ros2'")
    os.system("pkill -f 'op_ros2_agent'")
    os.system("pkill -f 'op_bridge_ros2'")
    os.system("pkill -f 'autoware'")
    time.sleep(3)
    os.system("pkill -f 'CarlaUE4'")
    time.sleep(1)
    os.system("pkill -f 'ros'")
    os.system("pkill -f 'CarlaUE4'")

if __name__ == "__main__":
    experiment_id = 'Cone'
    bag_id = '01'
    object_id = '5'
    ros_bag = os.path.join(experiment_id,bag_id)
    scenario='Car'
    targets =[] #roi_cluster_fusion
    params = {
    "X1": 0,
    "Y1": 0,
    "object_type": None
}
    params["X1"] = -1 
    params["Y1"] = 0
    # params["object_type"] = 'vehicle.tesla.model3'
    params["object_type"] = 'static.prop.streetbarrier'
    run_carla_scenario_agent(experiment_id,bag_id,scenario,params,targets)
    try:
        data_path = os.path.join(ObjectData_dir,ros_bag)
        if os.path.exists(data_path):
            shutil.rmtree(data_path)
        process_rosbag_file(ros_bag)
        print(f"processed rosbag '{ros_bag}")
    except Exception as e:
        print(f"Error processing rosbag '{ros_bag}': {str(e)}")
        run_carla_scenario_agent(experiment_id,bag_id,scenario,params,targets)
        time.sleep(1)
        process_rosbag_file(ros_bag)
    if bag_id == "01":
        object_id=record_failure(bag_id,experiment_id)
        param.object_id = object_id
        print(object_id)
    else:
        record_intervene(object_id,bag_id,experiment_id)