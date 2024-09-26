import networkx as nx
import matplotlib.pyplot as plt
import yaml
import pickle
node_topics = [
    {
        "Node_label": "lidar_centerpoint",
        "publish_topic": "/perception/object_recognition/detection/centerpoint/objects",
        "subscribe_topic":[] # "/sensing/lidar/concatenated/pointcloud"
    },
    {
        "Node_label": "detected_object_feature_remover",
        "publish_topic": "/perception/object_recognition/detection/clustering/camera_lidar_fusion/objects",
        "subscribe_topic": ["/perception/object_recognition/detection/clustering/camera_lidar_fusion/objects_with_feature"]
    },
    {
        "Node_label": "roi_cluster_fusion",
        "publish_topic": "/perception/object_recognition/detection/clustering/camera_lidar_fusion/clusters",
        "subscribe_topic": ["/perception/object_recognition/detection/clustering/clusters", 
                            "/perception/object_recognition/detection/rois0"]  # "/sensing/camera/camera0/camera_info"
    },
    {
        "Node_label": "fusion_shape_estimation",
        "publish_topic": "/perception/object_recognition/detection/clustering/camera_lidar_fusion/objects_with_feature",
        "subscribe_topic": ["/perception/object_recognition/detection/clustering/camera_lidar_fusion/clusters"]
    },
    {
        "Node_label": "clustering_shape_estimation",
        "publish_topic": "/perception/object_recognition/detection/clustering/objects_with_feature",
        "subscribe_topic": ["/perception/object_recognition/detection/clustering/clusters"]
    },
    {

        "Node_label": "euclidean_cluster",
        "publish_topic": "/perception/object_recognition/detection/clustering/clusters",
        "subscribe_topic":[] # /perception/object_recognition/detection/clustering/outlier_filter/pointcloud
    },
    
    # {
    #     "Node_label": "detection_by_tracker_node",
    #     "publish_topic": "/perception/object_recognition/detection/detection_by_tracker/objects",
    #     "subscribe_topic": ["/perception/object_recognition/detection/clustering/objects_with_feature"]
    # },
    # TODO: merge the object_association_merger_1 and detection_by_tracker_node, object_lanelet_filter
    {
        "Node_label": "object_association_merger_0",
        "publish_topic": "/perception/object_recognition/detection/camera_lidar_fusion/objects",
        "subscribe_topic":["/perception/object_recognition/detection/clustering/camera_lidar_fusion/objects",
                            "/perception/object_recognition/detection/centerpoint/validation/objects"] 
    },
    {
        "Node_label": "object_association_merger_1",
        "publish_topic": "/perception/object_recognition/detection/objects",
        "subscribe_topic": ["/perception/object_recognition/detection/camera_lidar_fusion/objects",
                            "/perception/object_recognition/detection/clustering/objects_with_feature"]
    },
    {
        "Node_label":"obstacle_pointcloud_based_validator",
        "publish_topic": "/perception/object_recognition/detection/centerpoint/validation/objects",
        "subscribe_topic": ["/perception/object_recognition/detection/centerpoint/objects"]
    },
    {
        "Node_label":"tensorrt_yolo",
        "publish_topic": "/perception/object_recognition/detection/rois0",
        "subscribe_topic": [] 
    },
    {
        "Node_label":"multi_object_tracker",
        "publish_topic": "/perception/object_recognition/tracking/objects",
        "subscribe_topic": ["/perception/object_recognition/detection/objects"]
    },
]

node_topics = [
    {
        "Node_label": "lidar_centerpoint",
        "publish_topic": "/perception/object_recognition/detection/centerpoint/objects",
        "subscribe_topic":[]  
    },
    {
        "Node_label": "clustering_shape_estimation",
        "publish_topic": "/perception/object_recognition/detection/clustering/objects_with_feature",
        "subscribe_topic": ["/perception/object_recognition/detection/clustering/clusters"]
    },
    {

        "Node_label": "euclidean_cluster",
        "publish_topic": "/perception/object_recognition/detection/clustering/clusters",
        "subscribe_topic":[]
    },
    {
        "Node_label": "object_association_merger_1",
        "publish_topic": "/perception/object_recognition/detection/objects",
        "subscribe_topic": [ "/perception/object_recognition/detection/centerpoint/validation/objects",
                            "/perception/object_recognition/detection/clustering/objects_with_feature"]
    },
    {
        "Node_label":"obstacle_pointcloud_based_validator",
        "publish_topic": "/perception/object_recognition/detection/centerpoint/validation/objects",
        "subscribe_topic": ["/perception/object_recognition/detection/centerpoint/objects"]
    },
    {
        "Node_label":"multi_object_tracker",
        "publish_topic": "/perception/object_recognition/tracking/objects",
        "subscribe_topic": ["/perception/object_recognition/detection/objects"]
    },
]

def get_node_labels_and_publish_topics(node_topics):
    print([node['Node_label'] for node in node_topics])
    return [(node['Node_label'], node['publish_topic']) for node in node_topics]

node_labels_and_publish_topics = get_node_labels_and_publish_topics(node_topics)
 
def create_dag(node_topics):
    G = nx.DiGraph()

    # Add nodes and edges to the graph
    for node in node_topics:
        node_label = node['Node_label']
        # In case the node publishes to multiple topics, treat each topic as a separate node
        publish_topic = node['publish_topic'] if isinstance(node['publish_topic'], list) else [node['publish_topic']]
        subscribe_topics = node['subscribe_topic']
        
        # Add the node with the label as the identifier
        if node_label not in G:
            G.add_node(node_label,label=node_label)
        else:
            G.nodes[node_label]['label']= node_label
        # Connect the node to the topics it subscribes to
        for sub_topic in subscribe_topics:
            # Find the label of the node that publishes the sub_topic
            publisher_node_label = [n['Node_label'] for n in node_topics if n['publish_topic'] == sub_topic]
            if publisher_node_label:  # Check if we found a publisher
                # Add edge from the publisher node label to the current node label
                G.add_edge(publisher_node_label[0], node_label)

    return G

def generate_domain_knowledge(G, filename):
    # domain_knowledge = generate_domain_knowledge(G, filename)
    root_nodes = [node for node, in_degree in G.in_degree() if in_degree == 0]
    leaf_nodes = [node for node, out_degree in G.out_degree() if out_degree == 0]
    requires = list(G.edges())
    # Forbids are simply the reverse of the requires
    # forbids = [(y, x) for x, y in requires]
    #     # Generate forbids by checking for non-direct paths in the DAG
    forbid_pairs = []
 
    all_nodes = G.nodes()
    for node_x in all_nodes:
        for node_y in all_nodes:
            if node_x != node_y:
                if not nx.has_path(G, node_x, node_y):
                    forbid_pairs.append((node_x, node_y))    
    domain_knowledge = {
        "root-nodes": root_nodes,
        "leaf-nodes": leaf_nodes,
        "forbids": forbid_pairs,
        "requires": requires
    }
    causal_graph = {
        "root-nodes": domain_knowledge["root-nodes"],
        "leaf-nodes": domain_knowledge["leaf-nodes"],
        "forbids": [list(edge) for edge in domain_knowledge["forbids"]],
        "requires": [list(edge) for edge in domain_knowledge["requires"]]
    }

    # Create the final structure to be saved to YAML
    final_structure = {"causal-graph": causal_graph}

    with open(filename, 'w') as file:
        yaml.dump(final_structure, file, default_flow_style=False)


def draw_dag(G):
    # Draw the graph
    pos = nx.spring_layout(G)  # positions for all nodes

    labels = nx.get_node_attributes(G, 'label')
    
    nx.draw(G, pos, with_labels=True, labels=labels, node_size=300, node_color='skyblue', font_size=10, font_weight='bold', arrows=True)
    plt.show()

def save_graph(G, filename):
    with open(filename, 'wb') as file:
        pickle.dump(G, file)
# Create the DAG from the node_topics
        
if __name__ == "__main__":        
    G = create_dag(node_topics)
    # Generate the domain knowledge from the DAG
    save_graph(G, 'LIDAR.pkl')
    # # Save the domain knowledge to a file named 'domain_knowledge.yml'
    draw_dag(G)