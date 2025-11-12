import torch
import torch.nn.functional as F
from torch.nn import Linear
from torch_geometric.nn import GATConv
from torch_geometric.nn import GATv2Conv

class GATv2Net(torch.nn.Module):
    def __init__(self, num_node_features, hidden_channels, output_channels=1):
        super(GATv2Net, self).__init__()
        self.conv1 = GATv2Conv(num_node_features, hidden_channels)
        # Output channels set to 1 for cost prediction
        # self.conv2 = GATv2Conv(hidden_channels, hidden_channels)
        self.conv2 = GATv2Conv(hidden_channels, output_channels)
        

    def forward(self, data):
        x, edge_index = data.x, data.edge_index

        # First GATv2 layer
        x = F.elu(self.conv1(x, edge_index))

        # Second GATv2 layer - directly outputs the cost prediction
        # x = F.elu(self.conv2(x, edge_index))
        
        x =  self.conv2(x, edge_index)

        return x
    
class GATv2NetClass(torch.nn.Module):
    def __init__(self, num_node_features, hidden_channels):
        super(GATv2NetClass, self).__init__()
        self.conv1 = GATv2Conv(num_node_features, hidden_channels)
        self.conv2 = GATv2Conv(hidden_channels, 1)  # Output channels set to 1 for binary classification

    def forward(self, data):
        x, edge_index = data.x, data.edge_index

        # First GATv2 layer with ELU activation
        x = F.elu(self.conv1(x, edge_index))

        # Second GATv2 layer - outputs the logit for being graspable
        x = self.conv2(x, edge_index)

        # Applying sigmoid activation to convert logits to probabilities
        x = torch.sigmoid(x)

        return x
