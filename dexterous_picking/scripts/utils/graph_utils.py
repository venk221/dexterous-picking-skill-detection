# graph_utils.py

import json
import math
import networkx as nx
import torch
import numpy as np
from torch_geometric.data import Data
from shapely.geometry import LineString, Polygon

def load_data_from_json(file_path):
    with open(file_path) as f:
        data = json.load(f)
    return data

def create_graph_from_data(data):
    G = nx.Graph()
    
    # data = data[1:3]
    
    # Add Wall node
    # G.add_node(0, position=(163, 540), skill=[0, 0, 0, 0, 0], skill_dir=(0, 0),
    #             bounding_box={'x1': 0, 'y1': 0, 'x2': 326, 'y2': 1080}, width_height=(326, 1080), is_wall=True)
    
    for i, obj in enumerate(data):
        
        
        node_id = i + 1
        width = obj['bounding_box']['x2'] - obj['bounding_box']['x1']
        height = obj['bounding_box']['y2'] - obj['bounding_box']['y1']
        skill_direction = (0, 0)

        if obj['skill'] == 4:
            skill_direction = (-1, 0)
            
            # skill_direction = (-1, 0)
            # skill_direction = (0, -1)
            # skill_direction = (-0.7071, -0.7071)
            # skill_direction = (-0.422618,-0.906308)
            
        elif obj['skill'] == 2:
            skill_direction = (1, 0)
            
        # Onehot encoding for skill
        # skill_one_hot = [0] * 5
        # skill_one_hot[obj['skill']-1] = 1
        
        skill_one_hot = [obj['skill']]

        G.add_node(node_id, position=(obj['position']['x'], obj['position']['y']), skill=skill_one_hot, skill_dir=skill_direction,
                bounding_box=obj['bounding_box'], width_height=(width, height), is_wall=False)

    # for i, obj1 in enumerate(data):
    #     for j, obj2 in enumerate(data):
    #         if i != j and has_visibility(obj1, obj2, [o for k, o in enumerate(data) if k not in [i, j]]):
    #             node_id_1 = i + 1
    #             node_id_2 = j + 1
    #             distance = round(bounding_box_distance(obj1['bounding_box'], obj2['bounding_box']), 2)
    #             G.add_edge(node_id_1, node_id_2, magnitude=distance)
                
    # Add edges between objects based on visibility
    for i, obj1 in enumerate(data):
        for j, obj2 in enumerate(data[i+1:], start=i+1):
            if has_visibility(obj1, obj2, [o for k, o in enumerate(data) if k not in [i, j]]):
                distance = round(bounding_box_distance(obj1['bounding_box'], obj2['bounding_box']), 2)
                G.add_edge(i + 1, j + 1, magnitude=distance)
        
                
    # Add edges from all objects to the wall node if the distance is less than 150 pixels
    # for i, obj in enumerate(data):
    #     node_id = i + 1
    #     distance = round(bounding_box_distance(obj['bounding_box'], G.nodes[0]['bounding_box']), 2)
    #     if distance < 150:
    #         G.add_edge(node_id, 0, magnitude=distance)

    return G

def bounding_box_distance(bb1, bb2):
    delta_x = max(0, max(bb1['x1'], bb2['x1']) - min(bb1['x2'], bb2['x2']))
    delta_y = max(0, max(bb1['y1'], bb2['y1']) - min(bb1['y2'], bb2['y2']))
    return math.sqrt(delta_x ** 2 + delta_y ** 2)

def has_visibility(obj1, obj2, others):
    # return True
    line_of_sight = LineString([(obj1['position']['x'], obj1['position']['y']), (obj2['position']['x'], obj2['position']['y'])])
    for other in others:
        other_bbox_polygon = Polygon([
            (other['bounding_box']['x1'], other['bounding_box']['y1']),
            (other['bounding_box']['x2'], other['bounding_box']['y1']),
            (other['bounding_box']['x2'], other['bounding_box']['y2']),
            (other['bounding_box']['x1'], other['bounding_box']['y2'])
        ])
        if line_of_sight.intersects(other_bbox_polygon):
            return False
    return True

def graph_to_data_object(G):
    node_features = []
    labels = []  # List to store grasping cost for each node as a label

    for node_id, attr in G.nodes(data=True):
        # print(attr.keys())
        
        pos = [attr['position'][0]/1920, attr['position'][1]/1080]
        skill = attr['skill']
        width_height = [attr['width_height'][0]/1920, attr['width_height'][1]/1080]
        skill_dir = [attr['skill_dir'][0], attr['skill_dir'][1]]
        node_feature = pos + width_height + skill + skill_dir

        node_features.append(node_feature)
        
        if not attr['is_wall']:
            # Assuming grasping_cost is stored directly in the node's attributes
             # Convert grasping cost to float
            labels.append(0.0)
        else:
            labels.append(-1.0)

    
    node_features_tensor = torch.tensor(node_features, dtype=torch.float)
    labels_tensor = torch.tensor(labels, dtype=torch.float).unsqueeze(-1)  # Convert labels list to a tensor

    edge_index = []
    edge_attr = []
    for start_node, end_node, attr in G.edges(data=True):
        # edge_index.append([start_node, end_node])
        edge_index.append([start_node-1, end_node-1])
        
        edge_attr.append([attr['magnitude']])

    # print(f"Edge index: {edge_index}, Edge index shape: {np.array(edge_index).shape}")
    edge_index_tensor = torch.tensor(edge_index, dtype=torch.long).t().contiguous()
    edge_attr_tensor = torch.tensor(edge_attr, dtype=torch.float)

    # Add labels_tensor as the y attribute in the Data object
    data = Data(x=node_features_tensor, edge_index=edge_index_tensor, edge_attr=edge_attr_tensor, y=labels_tensor)
    return data
