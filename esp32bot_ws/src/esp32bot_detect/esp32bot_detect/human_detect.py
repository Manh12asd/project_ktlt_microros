import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from cv_bridge import CvBridge
import cv2
import numpy as np

class HumanDetect(Node):
    def __init__(self):
        super().__init__('human_detect')
        self.subscription = self.create_subscription(
            Image,
            'droidcam/image_raw',
            self.listener_callback,
            10)
        self.image_publisher_ = self.create_publisher(Image, 'yolo/detections', 10)
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")  

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        
        results = self.model.predict(source=frame, save=False, conf=0.5, show=False)

        for result in results[0].boxes:
            x1, y1, x2, y2 = map(int, result.xyxy[0])
            conf = result.conf[0]
            cls = int(result.cls[0])
            if self.model.names[cls] in ["person"]:
                
                label = f"{self.model.names[cls]} {conf:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        self.image_publisher_.publish(image_msg)
    
def main(args=None):
    rclpy.init(args=args)
    human_detect = HumanDetect()
    rclpy.spin(human_detect)
    human_detect.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()