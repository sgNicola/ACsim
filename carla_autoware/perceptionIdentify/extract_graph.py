import pickle
import networkx as nx
import matplotlib.pyplot as plt
def load_graph(filename):
    with open(filename, 'rb') as file:
        return pickle.load(file)
    # Function to draw a graph, similar to draw_dag function

def draw_graph(G):
    pos = nx.spring_layout(G)
    labels = nx.get_node_attributes(G, 'label')
    nx.draw(G, pos, with_labels=True, labels=labels, node_size=300, node_color='skyblue', font_size=10, font_weight='bold', arrows=True)
    plt.show()

def create_subgraph(non_zero_nodes, G):
    """
    Create a subgraph from the original graph G using the list of non-zero nodes.
    Connect predecessors and successors of nodes that are not included in the non-zero nodes list.
    
    Parameters:
    - non_zero_nodes: A list of nodes with non-zero values.
    - G: The original graph from which the subgraph will be created.
    
    Returns:
    - subgraph: A NetworkX subgraph containing only the non-zero nodes and the necessary connections.
    """
    subgraph = G.subgraph(non_zero_nodes).copy()
    
    # For each node not in the non-zero nodes list
    for node in set(G.nodes()) - set(non_zero_nodes):
        # Get all predecessors and successors of the dropped node
        predecessors = set(G.predecessors(node))
        successors = set(G.successors(node))
        
        # For each predecessor, connect to all successors that are not already connected
        for pred in predecessors & set(subgraph.nodes()):
            for succ in successors & set(subgraph.nodes()):
                # Check if there is already a path from pred to succ, if not, connect them
                if not nx.has_path(subgraph, pred, succ):
                    subgraph.add_edge(pred, succ)
    return subgraph