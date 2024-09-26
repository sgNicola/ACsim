import networkx as nx
import os
# from run_fusion import run_carla_scenario_agent 
from run_lidar import run_carla_scenario_agent 
from perceptionIdentify.process_rosbag import process_rosbag_file
import time
from perceptionIdentify.utils import *
from perceptionIdentify.extract_graph import load_graph,create_subgraph
from Branch_pruning import get_ancestors, get_parents
# from perceptionIdentify.parse_fusion import record_failure
from perceptionIdentify.parse_lidar import record_failure
def GIWP(ini_P, F, experiment_id, object_id, id, scenario, params):
    C = set()
    X = set()
    while ini_P:
        P1 = get_first_half(ini_P)
        id += 1
        bag_id = f"{id:02d}"
        RP1 = run_and_process_rosbag(experiment_id, object_id, bag_id, scenario, params, P1)
        print("intervetion",P1)
        P2 = [p for p in ini_P if p not in P1]
        if F not in RP1:
            ini_P = P1
            if len(P1) == 1:
                C.update(P1)
                X.update(P2)
                print("Early stop: Required condition met")
                return C,X,id, True
            else:
                C_prime, X_prime, id,stop = GIWP(P1, F, experiment_id, object_id, id, scenario, params)
                C.update(C_prime)
                X.update(X_prime)
                if stop:
                    return C, X, id, True
        else:
            X.update(P1)
        for p in P2:
            if ((p in RP1) and (F not in RP1)) or ((p not in RP1) and (F in RP1)):
                X.add(p)
        ini_P = [p for p in ini_P if p not in X and p not in C]
        print("C",C,"X",X)
    return C, X, id, False

def BPI(G, experiment_id, object_id, F, scenario, params):
    C = set()
    X = set()
    id = 1
    nodes_order = list(nx.topological_sort(G))
    # Update C, X according to the branches found
    B =['multi_object_tracker']
    branch_order =[node for node in nodes_order if node not in B]
    C, X, id, stop = GIWP(branch_order, F, experiment_id, object_id, id, scenario, params)
    # C.update(C_prime)
    # X.update(X_prime)
    if stop:
        ordered_C = [node for node in nodes_order if node in C]
        return ordered_C, list(X), id
    C.add('multi_object_tracker')
    ordered_C = [node for node in nodes_order if node in C]
    return ordered_C, list(X), id

if __name__ == "__main__":
    experiment_id = 'InjectAID'
    bag_id = '01'
    ros_bag = os.path.join(experiment_id,bag_id)
    targets =[]
    scenario="Car"
    params = {
    "X1": 0,
    "Y1": 0,
    "object_type": None
}
    params["X1"] = -1 
    params["Y1"] = 0
    # params["object_type"] = 'static.prop.streetbarrier'
    params["object_type"] = 'vehicle.tesla.model3'
    #run_carla_scenario_agent(experiment_id,bag_id,scenario,params,targets)
    try:
        process_rosbag_file(ros_bag)
        print(f"processed rosbag '{ros_bag}")
    except Exception as e:
        print(f"Error processing rosbag '{ros_bag}': {str(e)}")
        run_carla_scenario_agent(experiment_id, bag_id,scenario,params,targets)
        time.sleep(1)
        process_rosbag_file(ros_bag)
    object_id=record_failure(bag_id,experiment_id)
    DAG = load_graph('LIDAR.pkl')
    bagname = '01'
    file_name = f'{bagname}_object_{object_id}_{failure_mode}.csv'
    file_id = os.path.join(experiment_id,file_name)
    file = os.path.join(Data_dir,file_id)
    # Create the subgraph from the DataFrame
    df = read_failure(file)
    causal_variables = get_non_zero_nodes(df)
    subgraph = create_subgraph(causal_variables, DAG)
    variables = df.columns[(df != 0).any()].tolist()
    F = 'multi_object_tracker'
    Chain,X,id= BPI(subgraph, experiment_id,object_id,F,scenario,params)
    C,X,id = GIWP(Chain,F,experiment_id,object_id,id,scenario,params)
    print(C,id)