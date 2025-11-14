import fvcore.nn.weight_init as weight_init
import torch
import torch.nn.functional as F
from torch import nn

from detectron2.layers import (
    CNNBlockBase,
    Conv2d,
    DeformConv,
    ModulatedDeformConv,
    ShapeSpec,
    get_norm,
)
from .resnet import build_resnet_backbone

from .backbone import Backbone
from .build import BACKBONE_REGISTRY

class DualResNetBackbone(Backbone):
    def __init__(self, cfg, input_shape: ShapeSpec):
        super().__init__()
        
        rgb_input_shape = ShapeSpec(channels=3, height=input_shape.height, width=input_shape.width)
        depth_input_shape = ShapeSpec(channels=3, height=input_shape.height, width=input_shape.width)
        
        # Instantiate two ResNet backbones, one for RGB and one for depth
        self.resnet_rgb = build_resnet_backbone(cfg, rgb_input_shape)
        self.resnet_depth = build_resnet_backbone(cfg, depth_input_shape)

        
        # Define 1D convolutional layers for feature fusion
        self.fusion_layers = nn.ModuleDict({
            feature_name: nn.Conv2d(
                in_channels=self.resnet_rgb.output_shape()[feature_name].channels * 2,
                out_channels=self.resnet_rgb.output_shape()[feature_name].channels,
                kernel_size=1,
                stride=1,
                padding=0
            ) for feature_name in self.resnet_rgb.output_shape()
        })
        
        self._out_features = cfg.MODEL.RESNETS.OUT_FEATURES
        self._out_feature_channels = {f: self.resnet_rgb.output_shape()[f].channels for f in self._out_features}
        self._out_feature_strides = {f: self.resnet_rgb.output_shape()[f].stride for f in self._out_features}
        
        
        # self.resnet_rgb.load_state_dict(torch.load(weights_rgb))
        # self.resnet_depth.load_state_dict(torch.load(weights_depth))

    def forward(self, x):
        
        # Split the input tensor into RGB and depth components
        # print(f"Forward size: {x.size()}")
        x_rgb, x_depth = torch.split(x, [3, 3], dim=1)
        
        # Forward pass through each ResNet
        features_rgb = self.resnet_rgb(x_rgb)
        features_depth = self.resnet_depth(x_depth)
        
        # Fuse and process features through 1D convolutions (using 2D conv with kernel_size=1 for channel-wise fusion)
        features_fused = {}
        for feature_name in self._out_features:
            combined = torch.cat([features_rgb[feature_name], features_depth[feature_name]], dim=1)
            fused = self.fusion_layers[feature_name](combined)
            features_fused[feature_name] = fused
        
        return features_fused

    def output_shape(self):
        return {name: ShapeSpec(channels=self._out_feature_channels[name], stride=self._out_feature_strides[name]) for name in self._out_features}


@BACKBONE_REGISTRY.register()
def build_dual_resnet_backbone(cfg, input_shape: ShapeSpec):
    return DualResNetBackbone(cfg, input_shape)