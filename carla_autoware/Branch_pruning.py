import networkx as nx
from param import failure_mode, Data_dir,ObjectData_dir
import os
import time
from perceptionIdentify.process_rosbag import process_rosbag_file
from perceptionIdentify.parse_fusion import record_failure
# from perceptionIdentify.parse_lidar import record_failure
from perceptionIdentify.extract_graph import load_graph, create_subgraph
from perceptionIdentify.utils import *
import shutil
def find_colliders(graph):
    # A collider has more than one incoming edge, so we look for nodes with in_degree > 1
    colliders = [node for node, degree in graph.in_degree() if degree > 1]
    return colliders

def is_collider(node, G):
    return len(list(G.predecessors(node))) > 1

def get_parents(node, G):
    return list(G.predecessors(node))

def get_ancestors(node, G):
    ancestors = set()
    # Recursive helper function to visit all ancestors
    def visit(n):
        # For each parent of the node, add to ancestors and visit it
        for parent in G.predecessors(n):
            if parent not in ancestors:  # Prevent infinite loops
                ancestors.add(parent)
                visit(parent)  # Recursively visit the parent
    # Initialize the recursive search
    visit(node)
    return ancestors

def bond_frozensets(C, M):
    bonded_sets = set()
    for m_set in M:
        intersection = m_set.intersection(C)
        if not intersection:
            bonded_sets.add(m_set)
    return bonded_sets

def get_set_in_B(B, M):
    for set_item in B:
        if set_item.issubset(M):
            return set(set_item)
    return None

def BPI(G,experiment_id, object_id,F,scenario,params):
    U = set()  # Set of nodes to be pruned
    M = set()    # Set of nodes to be merged
    id =1
    C = set()
    prune_graph = G.copy()
    priority_merges = []
    nodes_order = list(nx.topological_sort(G))
    for v in nodes_order:  # Assuming topological sort        
        targets =[]
        if is_collider(v, G):
            Pa = get_parents(v, G)
            p_star = min(Pa, key=lambda p: len(get_ancestors(p, G)))
            # Pa.remove(p_star)
            id +=1
            bag_id = f"{id:02d}"
            targets.append(p_star)
            causal_variables=run_and_process_rosbag(experiment_id,object_id,bag_id,scenario,params,targets)
            #causal_variables=get_causals(experiment_id,object_id,bag_id)
            p_star_ancestors = get_ancestors(p_star,prune_graph)
            if v not in causal_variables:
                U.add(v)                     
                if len(p_star_ancestors)==0:
                    C.add(p_star)
                else:
                    # priority_merges.append(p_star)
                    # priority_merges.extend(p_star_ancestors)
                    M.add(p_star)
                    M.update(p_star_ancestors)                 
            else:          
                C.add(v)
                U.add(p_star)
                U.update(p_star_ancestors)
                M.update(get_ancestors(v, prune_graph))
            prune_graph.remove_nodes_from(get_ancestors(v, G))
            E = [node for node in nodes_order if node not in U]
            for p in E:
                if ((p not in causal_variables) and (F in causal_variables)):
                    U.add(p)
    if M is not None:
        M.difference_update(U)
        M.difference_update(C)      
        M.update(prune_graph.nodes) 
        M.difference_update(U)
        M.difference_update(C)
    # priority_merges = [node for node in priority_merges if node not in U and node not in C]
    # ordered_M = priority_merges + [node for node in nodes_order if node in M and node not in priority_merges]
    ordered_M =[node for node in nodes_order if node in M]
    ordered_C = [node for node in nodes_order if node in C]
    print("Chain",ordered_M)
    #--------------------------------%%%%%%%------------------------
    return ordered_M,ordered_C,list(U),id

if __name__ == "__main__":
    experiment_id = 'Inject'
    bag_id = '01'
    ros_bag = os.path.join(experiment_id,bag_id)
    targets =[]
    scenario='Cone'
    params = {
    "X1": 0,
    "Y1": 0,
    "object_type": None
}
    params["X1"] = -1
    params["Y1"] = 0
    # params["object_type"] = 'static.prop.streetbarrier'
    params["object_type"] = 'vehicle.audi.tt'
    #run_carla_scenario_agent(experiment_id,bag_id,scenario,params,targets)
    try:
        data_path = os.path.join(ObjectData_dir,ros_bag)
        print(data_path)
        if os.path.exists(data_path):
            shutil.rmtree(data_path)
        process_rosbag_file(ros_bag)
        print(f"processed rosbag '{ros_bag}")
    except Exception as e:
        print(f"Error processing rosbag '{ros_bag}': {str(e)}")
        run_carla_scenario_agent(experiment_id, bag_id,scenario,params,targets)
        time.sleep(1)
        process_rosbag_file(ros_bag)
    object_id=record_failure(bag_id,experiment_id)
    # object_id = '21'
    # DAG = load_graph('DAG.pkl')
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
    Chain,C,U,id= BPI(subgraph, experiment_id,object_id,F,scenario,params)
    print(Chain,C,U)