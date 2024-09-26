import gc
from rcd import RCD
import os
from param import Data_dir, failure_mode,Rosbag_dir
from run_fusion import run_carla_scenario_agent 
from perceptionIdentify.process_rosbag import process_rosbag_file
from perceptionIdentify.extract_graph import load_graph, create_subgraph
from perceptionIdentify.utils import *
import time
from perceptionIdentify.parse_fusion import record_failure
import csv
import subprocess
import sys

def save_results_to_file(experiment_id, results):
    results_file = 'results.csv'    
    fieldnames = ['experiment_id', 'id','Roots']    
    # Check if the results file exists
    file_exists = os.path.isfile(results_file)    
    with open(results_file, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        
        # Write the header row if the file is newly created
        if not file_exists:
            writer.writeheader()
        
        # Append the data row
        writer.writerow({
            'experiment_id': experiment_id,
            'id':results['id'],
            'Roots': results['C'],
        })
    f.close()
    print(f"Results for experiment {experiment_id} appended to {results_file} successfully.")

def run_experiments(X1, Y1, object_types):
    # Check if there are saved values for i, j, l, and experiment_id
    if os.path.isfile("loop_values.txt"):
        with open("loop_values.txt", "r") as file:
            i, j, l = map(int, file.read().split(","))
        file.close()
    else:
        i, j, l = 0, 0, 0

    try:
        for x1 in X1[i:]:
            for y1 in Y1[j:]:
                for object_type in object_types[l:]:
                    try:
                        time.sleep(5)
                        experiment_id = f'{i * (len(Y1) * len(object_types)) + j * len(object_types) + l:02d}'
                        bag_id = '01'
                        ros_bag = os.path.join(experiment_id,bag_id)
                        targets =[]
                        params["X1"] = x1
                        params["Y1"] = y1
                        params["object_type"] = object_type
                        ROS_file = os.path.join(Rosbag_dir,ros_bag)
                        if not os.path.exists(ROS_file):
                            print("not exists",ros_bag)
                            run_carla_scenario_agent(experiment_id,bag_id,scenario,params,targets)
                        process_rosbag_file(ros_bag)
                        print(f"processed rosbag '{ros_bag}")
                        object_id=record_failure(bag_id,experiment_id)
                        DAG = load_graph('DAG.pkl')
                        file_name = f'{bag_id}_object_{object_id}_{failure_mode}.csv'
                        file_id = os.path.join(experiment_id,file_name)
                        file = os.path.join(Data_dir,file_id)
                        df = read_failure(file)
                        causal_variables = get_non_zero_nodes(df)
                        subgraph = create_subgraph(causal_variables, DAG)
                        F = 'multi_object_tracker'
                        C,id = RCD(subgraph, experiment_id, object_id, F, scenario, params)
                        results = {'C': C,'id':id}
                        os.system("pkill -f 'ros'")
                        os.system("pkill -f 'CarlaUE4'")
                        save_results_to_file(experiment_id, results)
                        l += 1
                        if l >= len(object_types):
                            l = 0
                            j += 1
                            if j >= len(Y1):
                                j = 0
                                i += 1
                        with open("loop_values.txt", "w") as file:
                            file.write(f"{i}, {j}, {l}")
                        subprocess.Popen([sys.executable, sys.argv[0]]) 
                        gc.collect()
                        sys.exit(0)
                    except RuntimeError as e:
                        print(f"Error running carla_scenario_agent for experiment ID '{experiment_id}': {str(e)}")
                        # Clean up resources
                        os.system("pkill -f 'ros'")
                        os.system("pkill -f 'CarlaUE4'")
                        time.sleep(10)
                        subprocess.Popen([sys.executable, sys.argv[0]]) 
                        sys.exit(0)
                    except Exception as e:
                        print(f"Error running carla_scenario_agent for experiment ID '{experiment_id}': {str(e)}")
                        # Clean up resources
                        os.system("pkill -f 'ros'")
                        os.system("pkill -f 'CarlaUE4'")
                        l += 1
                        if l >= len(object_types):
                            l = 0
                            j += 1
                            if j >= len(Y1):
                                j = 0
                                i += 1
                        with open("loop_values.txt", "w") as file:
                            file.write(f"{i}, {j}, {l}")
                        time.sleep(10)
                        subprocess.Popen([sys.executable, sys.argv[0]]) 
                        sys.exit(0)
        os.remove("loop_values.txt")
    except Exception as e:
        print(f"Error occurred: {str(e)}")

if __name__ == "__main__":
    # scenario="Car"
    #%%%%%%%%%%%%%%%%
    scenario="Cone"
    X1 = [-1,0,1,3,5,7,9,-3,2,4,8,10,-2,6]
    Y1 = [-1,-0,1,-0.5]
    object_types=['static.prop.streetbarrier',
        'static.prop.constructioncone',
        'static.prop.trafficcone01',
        'static.prop.trafficcone02',
        'static.prop.warningconstruction',
        'static.prop.warningaccident',
        ]
    params = {
        "X1": 0,
        "Y1": 0,
        "object_type": None
    }
    run_experiments(X1, Y1, object_types)
    # X1 = [-10,0,-5,-7,-9,-3,-2,-4,-8,-6]
    # Y1 = [0,-1,0.6]
    # object_types=['vehicle.tesla.model3',
    #     'vehicle.toyota.prius',
    #     'vehicle.audi.tt',
    #     'vehicle.jeep.wrangler_rubicon'
    #     ]
    # params = {
    #     "X1": 0,
    #     "Y1": 0,
    #     "object_type": None
    # }
    # run_experiments(X1, Y1, object_types)    