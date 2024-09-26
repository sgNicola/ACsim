import networkx as nx
# from run_fusion import run_carla_scenario_agent 
from perceptionIdentify.utils import *

import random

def divide_into_random_sets(U, num_sets):
    original_indices = {value: idx for idx, value in enumerate(U)}
    random.shuffle(U)  # Shuffle the list randomly
    set_size = len(U) // num_sets  # Calculate the approximate size of each set
    remainder = len(U) % num_sets  # Calculate the remaining elements
    sets = []
    start = 0
    for i in range(num_sets):
        end = start + set_size + (1 if i < remainder else 0)  # Adjust set size for the remaining elements
        subset = U[start:end]
        subset.sort(key=lambda x: original_indices[x])  # Sort subset based on the original order in U
        sets.append(subset)
        start = end
    return sets

def RCD(G, experiment_id, object_id, F, scenario, params):
    k=2
    C = set()
    M = []
    id = 1
    U = list(nx.topological_sort(G))
    num_sets = 3  # Number of sets to divide into
    if len(U)>k: 
        random_sets = divide_into_random_sets(U, num_sets)
        for subset in random_sets:
            filtered_subset = [node for node in subset if node in U]
            if filtered_subset:
                id += 1
                F_node = filtered_subset[0]
                F_list = [F_node]
                bag_id = f"{id:02d}"
                RP1 = run_and_process_rosbag(experiment_id, object_id, bag_id, scenario, params, F_list)
                if F not in RP1:
                    C.add(F_node)
                else:
                    for node in filtered_subset:
                        if node in RP1:
                            M.append(node)
    else:
        if U:
            C.add(U[0])
    if len(C) >= k:
        return list(C)[:k], id
    else:
        while M and len(C) < k:
            F_node = M.pop(0)
            F_list = [F_node]
            id += 1
            bag_id = f"{id:02d}"
            RP1 = run_and_process_rosbag(experiment_id, object_id, bag_id, scenario, params, F_list)
            if F not in RP1:
                C.add(F_node)
            else:
                for node in M:
                    if node in RP1:
                        C.add(node)
                        # if len(C) == k:
                        #     break
        return list(C)[:k], id   