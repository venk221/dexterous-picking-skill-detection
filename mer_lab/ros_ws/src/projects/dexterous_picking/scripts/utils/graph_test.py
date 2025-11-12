import torch
from network import GATv2Net, GATv2NetClass  # Assuming this is your model class
from utils.graph_utils import create_graph_from_data, graph_to_data_object, load_data_from_json
import os
import json
from PIL import Image, ImageDraw, ImageFont
import matplotlib.pyplot as plt
import os
from pathlib import Path
import cv2
import numpy as np

# from torch_geometric.explain import Explainer, AttentionExplainer
from torch_geometric.explain.metric import unfaithfulness, fidelity

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


def load_model(model_path, num_features):
    """
    Load the trained model from the specified path.
    """
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = GATv2Net(num_node_features=num_features, hidden_channels=64, output_channels=1)
    # model = GATv2NetClass(num_node_features=num_features, hidden_channels=64)
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval()
    model.to(device)
    return model

def predict_execution_order(model, data_object):
    """
    Predict the execution order for the nodes in a graph.
    """
    with torch.no_grad():
        out = model(data_object)
    return out

def plot_bounding_boxes(image_path, data, order, output_dir):
    """
    Plot bounding boxes on the image with execution order using cv2 and save to output directory.
    """
    # Load the image
    img = cv2.imread(image_path)
    
    # Specify font scale and thickness
    font_scale = 0.9
    thickness = 2
    font = cv2.FONT_HERSHEY_SIMPLEX

    for i, node_id in enumerate(order):
        node_data = data[node_id]
        # node_data = data[i]
        bbox = node_data['bounding_box']
        print(node_data)
        
        # OpenCV uses BGR format, and the rectangle coordinates need to be in integers
        top_left = (int(bbox['x1']), int(bbox['y1']))
        bottom_right = (int(bbox['x2']), int(bbox['y2']))
        cv2.rectangle(img, top_left, bottom_right, (0, 0, 255), 2)  # Red rectangle with thickness of 2
        
        # Calculate text size to adjust positioning
        text = str(i + 1)
        # text = f"{node_id}"
        (text_width, text_height), _ = cv2.getTextSize(text, font, fontScale=font_scale, thickness=thickness)
        
        # Position text slightly above the top left corner of the bounding box
        text_offset_x = top_left[0]
        text_offset_y = top_left[1] - 10  # Adjust this value to avoid overlap with the bounding box
        text_bottom_left_corner = (text_offset_x, text_offset_y)
        
        # Add background for text to improve readability
        img = cv2.rectangle(img, (text_offset_x, text_offset_y - text_height - 3), (text_offset_x + text_width, text_offset_y + 3), (0, 0, 255), -1)
        img = cv2.putText(img, text, text_bottom_left_corner, font, font_scale, (255, 255, 255), thickness)

    # Define output path
    output_path = os.path.join(output_dir, f"annotated_{os.path.basename(image_path)}")
    
    # Save the edited image
    cv2.imwrite(output_path, img)

def main(graph_json_path, model_path, image_path, output_dir):
    """
    Main function to load a graph, predict the execution order, and save the annotated image.
    """
    # Load graph data
    data = load_data_from_json(graph_json_path)
    G = create_graph_from_data(data)
    data_object = graph_to_data_object(G).to(device)
    
    num_features = 11  # Assuming your model takes 7 features
    model = load_model(model_path, num_features)
    
    print(f"Data object: {data_object.x}")

    # Predict execution order
    predictions = predict_execution_order(model, data_object)[1:]
    print(predictions)
    # predictions = [1 if x > 0.5 else 0 for x in predictions]
    # print(predictions)
    # Sort nodes by predicted values to get the execution order
    # order = sorted(range(len(predictions)), key=lambda i: predictions[i])
    order = sorted(range(len(predictions)), key=lambda i: predictions[i].item())
    order = order[::-1]  # Reverse the order to get the highest value first
    print(order)

    # Plot bounding boxes with execution order on the image and save it
    plot_bounding_boxes(image_path, data, order, output_dir)
    # plot_bounding_boxes(image_path, data, predictions, output_dir)
    
    
    # explainer = explainer = Explainer(
    #     model=model,
    #     algorithm=AttentionExplainer(model),
    #     explanation_type='model',
    #     node_mask_type='attributes',
    #     model_config=dict(
    #         mode='regression',
    #         task_level='node',
    #         return_type='raw',
    #     ),
    # )
    # node_idx = 0  # Node index for which to explain the attention (can loop over nodes)
    # node_feat_mask, edge_mask = explainer.explain_node(node_idx, data_object.x, data_object.edge_index)

    # # Visualize the explanation for the specified node
    # ax, G = explainer.visualizvisualize_feature_importancee_subgraph(node_idx, data_object.edge_index, edge_mask, y=predictions)
    # # plt.show()

    # # Save the explanation visualization
    # explainer.save_fig('attention_explanation.png')

if __name__ == "__main__":
    graph_json_path = 'NewData/Test/scene1/updated.json'  # Update this path
    # model_path = 'models/model_regression_lr0.005_batch_8_1000epochs.pth'  # Update this path
    model_path = 'model.pth'  # Update this path
    image_path = '/home/mpdeshmukh/Graph/NewData/Test/scene1/rgb_image.jpg'  # Update this path
    output_dir = '/home/mpdeshmukh/Graph/outputs'  # Define the output directory
    main(graph_json_path, model_path, image_path, output_dir)
