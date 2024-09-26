import networkx as nx
from Branch_pruning import BPI
import os
# from run_fusion import run_carla_scenario_agent 
from run_lidar import run_carla_scenario_agent
from perceptionIdentify.process_rosbag import process_rosbag_file
import time
from perceptionIdentify.utils import *
from perceptionIdentify.extract_graph import load_graph,create_subgraph
from perceptionIdentify.parse_lidar import record_failure
def GIWP_I(P, F, experiment_id, object_id, id,scenario,params):
    C = []   
    X = []   
    while P:
        P1 = get_first_half(P)
        id += 1
        bag_id = f"{id:02d}"
        RP1 = run_and_process_rosbag(experiment_id, object_id, bag_id, scenario,params,P1)
        #RP1 = get_causals(experiment_id, object_id, bag_id)
        # Check if the failure stops after intervention
        P2 = [p for p in P if p not in P1]
        for p in P[:]:  
            if ((p in RP1) and (F not in RP1)) or ((p not in RP1) and (F in RP1)):
                if p not in X: X.append(p)
                P.remove(p)
                if p in P2: P2.remove(p)
        if F not in RP1:
            if len(P1) == 1:
                if P1[0] not in C: C.append(P1[0])
                return C, X, id
            else:
                # Recursive call to further investigate the first half
                C_prime, X_prime, id = GIWP_I(P1, F, experiment_id, object_id, id,scenario,params)
                for item in C_prime:
                    if item not in C: C.append(item)  
                for item in X_prime:
                    if item not in X: X.append(item)
                return C, X, id
        if F in RP1:
            if any(p not in RP1 for p in P2):
                C_prime, X_prime, id = GIWP_I(P2, F, experiment_id, object_id, id,scenario,params)
                for item in C_prime:
                    if item not in C: C.append(item)  
                for item in X_prime:
                    if item not in X: X.append(item)
                F = P2[0]
                C_star, X_star, id = GIWP_I(P1, F, experiment_id, object_id, id,scenario,params)
                for item in C_star:
                    if item not in C: C.append(item)  
                for item in X_star:
                    if item not in X: X.append(item)
                return C, X, id
            else:
                for item in P1:
                    if item not in X: X.append(item)
        CX = C + X 
        P = [p for p in P if p not in CX]

    return C, X, id

def GIWP(ini_P, F, experiment_id, object_id, id,scenario,params):
    C = []   
    X = []
    while ini_P:
        P1 = get_first_half(ini_P)
        id += 1
        bag_id = f"{id:02d}"
        RP1 = run_and_process_rosbag(experiment_id, object_id, bag_id, scenario,params,P1)
        P2 = [p for p in ini_P if p not in P1]
        if F not in RP1:
            ini_P=P1
            if len(P1) == 1:
                C.extend(P1)
                X.extend(P2)
            else:
                C_prime, X_prime, id = GIWP(ini_P, F, experiment_id, object_id, id,scenario,params)
                C.extend(item for item in C_prime if item not in C)
                X.extend(item for item in X_prime if item not in X)                                
        else:
            if len(P2) == 1:
                C.extend(P2)
            else:        
                ini_P = P2        
                C_prime, X_prime, id = GIWP(ini_P, F, experiment_id, object_id, id,scenario,params)
                C.extend(item for item in C_prime if item not in C)  
                X.extend(item for item in X_prime if item not in X)
            if len(P1) == 1:
                C.extend(P1)
            else:
                F = P1[-1]
                ini_P = P1
                C_prime, X_prime, id = GIWP(ini_P, F, experiment_id, object_id, id,scenario,params)
                C.extend(item for item in C_prime if item not in C)
                X.extend(item for item in X_prime if item not in X)               
            for p in P2:  
                if ((p in RP1) and (F not in RP1)) or ((p not in RP1) and (F in RP1)):
                    if p not in X:
                        X.append(p)
        ini_P = [p for p in ini_P if p not in X and p not in C]
    return C, X, id,

if __name__ == "__main__":
    experiment_id = 'Inject'
    bag_id = '01'
    ros_bag = os.path.join(experiment_id,bag_id)
    targets =[]
    scenario="Cone"
    params = {
    "X1": 0,
    "Y1": 0,
    "object_type": None
}
    params["X1"] = 1 
    params["Y1"] = 0
    # params["object_type"] = 'static.prop.streetbarrier'
    params["object_type"] = 'vehicle.jeep.wrangler_rubicon'
    # Example usage
    run_carla_scenario_agent(experiment_id,bag_id,scenario,params,targets)
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
    Chain,R,U,id= BPI(subgraph, experiment_id,object_id,F,scenario,params)
    # print(Chain)
    C, X,id = GIWP(Chain,F,experiment_id,object_id,id,scenario,params)
    Roots = C+R
    print(C,R,Roots,id)