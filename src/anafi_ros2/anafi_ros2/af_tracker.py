import rclpy
import cv2
import os
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from anafi_msg.msg import BoxData
from anafi_ros2.lib.Sort import Sort

from cv_bridge import CvBridge
from detectron2.utils.logger import setup_logger
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg

setup_logger()

class DroneTracker(Node):
    def __init__(self):
        super().__init__('af_tracker')

        #LAUNCH PARAMETERS  
        self.declare_parameter('display_2D', 'false')
        self.display = self.get_parameter('display_2D').get_parameter_value().bool_value

        #PUBLISHERS / SUBSCRIBERS
        self.frames_sub = self.create_subscription(Image,'anafi/frames', self.frame_callback, 1)
        self.bbox_pub = self.create_publisher(BoxData, 'anafi/bbox2D',1)
        self.bridge = CvBridge()

        #MASK R-CNN MODEL
        self.cfg = get_cfg()
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        self.cfg.DATASETS.TRAIN = ('drone_train',)
        self.cfg.DATASETS.TEST = ('drone_test',)
        self.cfg.DATALOADER.NUM_WORKERS = 2
        self.cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = 128
        self.cfg.MODEL.ROI_HEADS.NUM_CLASSES = 2
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.95
        self.cfg.MODEL.DEVICE = "cuda"
        self.cfg.SOLVER.BASE_LR = 0.0025 
        self.cfg.SOLVER.IMS_PER_BATCH = 2
        self.cfg.MODEL.WEIGHTS = os.path.join("./src/anafi_ros2/anafi_ros2/Data/model_final.pth") #path to .pth file
        self.predictor = DefaultPredictor(self.cfg)

        #SORT ALGORITH
        self.mot_tracker = Sort(max_age=200, min_hits=20, iou_threshold=0.1)

        #VISUALIZATION
        self.colours = np.random.rand(200, 3)
        self.fps_time = 0.0

    def frame_callback(self, msg):
        
        data = None

        frameid = msg.header.frame_id
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        outputs = self.predictor(cv_image)
        data = self.tracker_props(outputs)
        self.bboxPublish(data, frameid)
        
        if self.display:
            if data is not None:
                for bbox in data:
                    id = int(bbox[4])
                    colours = (self.colours*255).astype(int)
                    B, G, R = int(colours[id,0]), int(colours[id,1]), int(colours[id,2])
                    x1, y1, x2, y2 = map(int, [bbox[0], bbox[1], bbox[2], bbox[3]])
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (B,G,R), 2)
                    cv2.putText(cv_image, f"ID: {id}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (B, G, R), 2)
                
            cv2.imshow('Display', cv_image)
            cv2.waitKey(1) & 0xFF == ord('0')
    
    def tracker_props(self, predictions):
        
        ins_data = []
        instances = predictions["instances"]

        if len(instances.pred_masks) >=1:

            for n in range(len(instances.pred_masks)):
                box = instances.pred_boxes.tensor[n].cpu().tolist()
                score = instances.scores[n].cpu().tolist()
                box.append(round(score,4))
                ins_data.append(box)
            ins_data = np.array(ins_data)

        else:
            ins_data = np.empty((0, 5))

        new_box = self.mot_tracker.update(ins_data)


        return new_box.tolist()
    
    
    def bboxPublish(self,detections, frameid):

        msg = BoxData()
        msg.frame_id = int(frameid)
        flattened_data = []
        
        if len(detections)>=1:
            msg.target = True
            flattened_data.extend([int(item) for sublist in detections for item in sublist])
        else:
            msg.target = False

        msg.data = flattened_data
        self.bbox_pub.publish(msg)

    def Stop(self):
        self.display = False

        cv2.destroyAllWindows()
        self.frames_sub.destroy()

        self.destroy_node()
        rclpy.shutdown()
        

def main(args=None):
    rclpy.init(args=args)
    af_tracker = DroneTracker()

    try:
        while rclpy.ok():
            rclpy.spin_once(af_tracker)
    except: 
        af_tracker.Stop()

if __name__ == '__main__':
    main()
