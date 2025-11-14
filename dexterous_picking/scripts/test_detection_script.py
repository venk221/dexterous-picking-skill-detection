#!/usr/bin/env python
import rospy
from detectron2.config import get_cfg
from cv_bridge import CvBridge
# import some common detectron2 utilities
from detectron2.engine import DefaultPredictor
from detectron2.utils.logger import setup_logger
from detectron2.utils.visualizer import Visualizer
from dexterous_picking.srv import GetGrasp, GetGraspResponse
from sensor_msgs.msg import Image
import rospkg
import cv2

color_dict = {1: ['Flip', (0,128,255)],
                2: ['Push-to-horizontal', (255,0,255)],
                3: ['Push-to-vertical', (255,51,153)],
                4: ['Simple-pick', (128,255,0)],
                5: ['Slide-to-edge', (0,0,255)]}

class Detectron2(object):
    def __init__(self):
        setup_logger()
        self.last_msg = None

        rospack = rospkg.RosPack()
        pkgdir = rospack.get_path('dexterous_picking')
        self.cfg = get_cfg()
        self.cfg.merge_from_file(pkgdir + "/config/mask_rcnn_R_50_FPN_3x.yaml")  # Update with the path to the config file used for training
        self.cfg.MODEL.WEIGHTS = pkgdir + "/config/" + "model_new_sim.pth"
        self.cfg.MODEL.DEVICE = "cpu"
        self.cfg.MODEL.ROI_HEADS.NUM_CLASSES = 5
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.7
        self.predictor = DefaultPredictor(self.cfg)

        self.visualization = True
    
    def get_object_labels(self, img):

        outputs = self.predictor(img)
        result = [value.item() for value in outputs["instances"].pred_classes]

        instances = outputs["instances"].to("cpu")

        if "scores" in instances._fields:
            conf_scores = instances.scores.tolist()
        else:
            raise AttributeError("Unable to find 'scores' in the Instances object.")
        # from IPython import embed; embed()
        vis_img = self.create_visualization(img, instances)

        # print(conf_scores)
        # from IPython import embed; embed()

        # # Visualize results
        # if self.visualization:
        #     v = Visualizer(img[:, :, ::-1], scale=1.2)
        #     v = v.draw_instance_predictions(instances)
        #     vis_img = v.get_image()[:, :, ::-1]
        return vis_img
    
    def create_visualization(self, img, instances):
        for i in range(len(instances)):
            instance = instances[i]
            box = instance.pred_boxes.tensor.numpy()[0].astype(int)
            # For bounding box
            img = cv2.rectangle(img, (box[0], box[1]), (box[2], box[3]), color_dict[instance.pred_classes.numpy()[0]][1], 2)
            center = (int((box[0]+box[2])/2), int((box[1]+box[3])/2))
            # For the text background
            # Finds space required by the text so that we can put a background with that amount of width.
            label = color_dict[instance.pred_classes.numpy()[0]][0] + " " + str(int(instance.scores.numpy()[0]*100)) + "%"
            (w, h), _ = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)

            # Prints the text.    
            img = cv2.rectangle(img, (center[0]-int(w/2), center[1] - 30), (center[0]+int(w/2),  center[1]-10), color_dict[instance.pred_classes.numpy()[0]][1], -1)
            img = cv2.putText(img, label, (center[0]-int(w/2), center[1] - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)
        return img


def main(argv):
    img = cv2.imread("/home/merlab/mer_lab/ros_ws/src/projects/dexterous_picking/scripts/depth_image.jpg")
    node = Detectron2()
    vis_img = node.get_object_labels(img)
    cv2.imshow("test", vis_img)
    cv2.waitKey(0)

if __name__ == '__main__':
    main(sys.argv)
