import xml.etree.ElementTree as ET
def find_parent(element, tree):
    for parent in tree.iter():
        if element in list(parent):
            return parent
    return None

def get_node_info(node_labels):
    nodes_info = []
    node_files = [
    {
        "Node_label": "lidar_centerpoint",
        "namespace" :"centerpoint",
        "include_file": "$(find-pkg-share lidar_centerpoint)/launch/lidar_centerpoint.launch.xml"
    },
    {
        "Node_label": "detected_object_feature_remover",
        "namespace" :"camera_lidar_fusion",
        "include_file": "$(find-pkg-share detected_object_feature_remover)/launch/detected_object_feature_remover.launch.xml"
    },
    {
        "Node_label": "roi_cluster_fusion",
        "namespace" :"camera_lidar_fusion",
        "include_file":"$(find-pkg-share image_projection_based_fusion)/launch/roi_cluster_fusion.launch.xml"
    },
    {
        "Node_label": "clustering_shape_estimation",
        "namespace" :"clustering",
        "include_file":"$(find-pkg-share shape_estimation)/launch/shape_estimation.launch.xml"
    },
    {
        "Node_label": "fusion_shape_estimation",
        "namespace" :"camera_lidar_fusion",
        "include_file":"$(find-pkg-share shape_estimation)/launch/shape_estimation.launch.xml"
    },

    {
        "Node_label": "detection_by_tracker",
        "namespace" :"",
        "include_file": "$(find-pkg-share detection_by_tracker)/launch/detection_by_tracker.launch.xml"
    },
    {
        "Node_label": "euclidean_cluster",
        "namespace" :"clustering",
        "include_file": "$(find-pkg-share euclidean_cluster)/launch/voxel_grid_based_euclidean_cluster.launch.xml"
 
    },
    {
        "Node_label": "object_association_merger_0",
        "namespace" :"",
        "include_file":"$(find-pkg-share object_merger)/launch/object_association_merger.launch.xml",
        "args": {
                "input/object1": "clustering/camera_lidar_fusion/objects",
            }
    },
    {
        "Node_label": "object_association_merger_1",
        "namespace" :"",
        "include_file":"$(find-pkg-share object_merger)/launch/object_association_merger.launch.xml",
        "args": {
                "input/object0": "camera_lidar_fusion/objects",
                "input/object1": "detection_by_tracker/objects",
            }
    },
    {
        "Node_label":"obstacle_pointcloud_based_validator",
        "namespace" :"",
        "include_file":"$(find-pkg-share detected_object_validation)/launch/obstacle_pointcloud_based_validator.launch.xml"
    },
    {
        "Node_label":"tensorrt_yolo",
        "namespace" :"",
        "include_file":"$(find-pkg-share tensorrt_yolo)/launch/yolo.launch.xml"
    }
] 
    for node_label in node_labels:
        for node in node_files:
            if node["Node_label"] == node_label:
                nodes_info.append(node)
                break
    return nodes_info

def comment_out_yolo(xml_root, node):
    include_file = node["include_file"]
    for include in xml_root.findall(f".//include[@file='{include_file}']"):
        include_str = ET.tostring(include, encoding='unicode').strip()
        comment = ET.Comment(include_str)
        parent = find_parent(include, xml_root)
        if parent is not None:  # Check if the parent element was found
            parent_index = list(parent).index(include)
            parent.remove(include)
            parent.insert(parent_index, comment)

def comment_out_group(xml_root, node):
        include_file = node["include_file"]
        # Iterate over all 'group' elements in the XML
        args = node.get("args", {})
        for group in xml_root.findall(".//group"):
            include = group.find(f".//include[@file='{include_file}']")
            if include is None:
                continue  # If include element is not found, skip to the next group
            if args:
                all_args_match = all(
                    include.find(f".//arg[@name='{arg_name}'][@value='{arg_value}']") is not None
                    for arg_name, arg_value in args.items()
                )
                if not all_args_match:
                    continue  
                # Serialize the target_group to a string and create a comment
            target_group_str = ET.tostring(group, encoding='unicode').strip()
            comment = ET.Comment(target_group_str)
            parent = find_parent(group, xml_root)
            parent_index = list(parent).index(group)
            parent.remove(group)
            parent.insert(parent_index, comment)
            break

def comment_out_namespace(xml_root, node):
        include_file = node["include_file"]
        namespace = node["namespace"]
        # Iterate over all 'group' elements in the XML
        for group in xml_root.findall(".//group"):
            push_ros_namespace = group.find(f".//push-ros-namespace[@namespace='{namespace}']")
            if push_ros_namespace is None:
                continue
            for subgroup in group.findall(".//group"):
                include = subgroup.find(f".//include[@file='{include_file}']")
                if include is None:
                    continue  # If include element is not found, skip to the next group
                    # Serialize the target_group to a string and create a comment
                target_group_str = ET.tostring(subgroup, encoding='unicode').strip()
                comment = ET.Comment(target_group_str)
                parent = find_parent(subgroup, xml_root)
                parent_index = list(parent).index(subgroup)
                parent.remove(subgroup)
                parent.insert(parent_index, comment)
                break 

def comment_out_fusion(xml_root, node):
        include_file = node["include_file"]
        namespace = node["namespace"]
        ros_namespace = "clustering"
        # Iterate over all 'group' elements in the XML
        for group in xml_root.findall(".//group"):
            clustering_namespace = group.find(f".//push-ros-namespace[@namespace='{ros_namespace}']")
            if clustering_namespace is None:
                continue
            for subgroup in group.findall(".//group"):
                push_ros_namespace = group.find(f".//push-ros-namespace[@namespace='{namespace}']")
                if push_ros_namespace is None:
                    continue
                for subsubgroup in subgroup.findall(".//group"):
                    include = subsubgroup.find(f".//include[@file='{include_file}']")
                    if include is None:
                        continue  # If include element is not found, skip to the next group
                        # Serialize the target_group to a string and create a comment
                    target_group_str = ET.tostring(subsubgroup, encoding='unicode').strip()
                    comment = ET.Comment(target_group_str)
                    parent = find_parent(subsubgroup, xml_root)
                    parent_index = list(parent).index(subsubgroup)
                    parent.remove(subsubgroup)
                    parent.insert(parent_index, comment)
                    break 

def comment_out_group_with_include(xml_root, node_labels):
        # Define the unique identifiers for each group
    nodes = get_node_info(node_labels)
    for node in nodes:        
        if node["Node_label"] == "tensorrt_yolo":
            comment_out_yolo(xml_root, node)
        if node["namespace"] == "":
            comment_out_group(xml_root, node)
        if node["namespace"] == "clustering":
            comment_out_namespace(xml_root, node)
        if node["namespace"] == "camera_lidar_fusion":
            comment_out_fusion(xml_root, node)
        if node["namespace"] == "centerpoint":
            comment_out_namespace(xml_root, node)


def modify_and_save_xml(input_file_path, output_file_path, node_files):
    # Parse the XML content from file
    tree = ET.parse(input_file_path)
    root = tree.getroot()

    # Comment out the specific group
    comment_out_group_with_include(
        root, node_files
    )
    tree.write(output_file_path, encoding='utf-8', xml_declaration=True)

if __name__ == "__main__":
    # Path to the original XML file
    input_file_path = '/home/anonymous/ACsim/carla_autoware/camera_lidar_fusion_based_detection.launch.xml'
    output_file_path = '/home/anonymous/ACsim/carla_autoware/test.launch.xml'
    node_labels = ["detected_object_feature_remover"]
    modify_and_save_xml(input_file_path, output_file_path, node_labels)
