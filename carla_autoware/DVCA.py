import networkx as nx
import os
# from run_fusion import run_carla_scenario_agent 
from run_fusion import run_carla_scenario_agent
from perceptionIdentify.process_rosbag import process_rosbag_file
import time
from perceptionIdentify.utils import *
from perceptionIdentify.extract_graph import load_graph,create_subgraph
from Branch_pruning import get_ancestors, get_parents
from perceptionIdentify.parse_fusion import record_failure

def DVCA(G, experiment_id, object_id, F, scenario, params):
    C = set()
    X = set()
    id = 1
    nodes = list(nx.topological_sort(G))
    left =0
    right= len(nodes)-1
    last_node_before_update = None

    while left < right:
        id += 1
        bag_id = f"{id:02d}"
        mid = (left + right) // 2  # Find the midpoint for binary search
        P1 = nodes[left:mid+1]
        RP1 = run_and_process_rosbag(experiment_id, object_id, bag_id, scenario, params, P1)
        # If failure F is not in the results from the left half, the issue is in the right half
        if F not in RP1: 
            last_node_before_update = nodes[left]
            left = mid + 1 # right half
        else:
            last_node_before_update = nodes[right]
            right = mid   # Move right marker to mid 
    if last_node_before_update is None:
        last_node_before_update=nodes[left]
    return last_node_before_update, id

if __name__ == "__main__":
    experiment_id = 'DVCA'
    bag_id = '01'
    ros_bag = os.path.join(experiment_id,bag_id)
    targets =[]
    scenario="Cone"
    params = {
    "X1": 0,
    "Y1": 0,
    "object_type": None
}
    params["X1"] = -1 
    params["Y1"] = 0
    # params["object_type"] = 'static.prop.streetbarrier'
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
    DAG = load_graph('DAG.pkl')
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
    C,id = DVCA(subgraph, experiment_id, object_id, F, scenario, params)
    print(C,id)