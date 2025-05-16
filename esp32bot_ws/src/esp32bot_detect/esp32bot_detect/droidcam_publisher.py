import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DroidCamPublisher(Node):
    def __init__(self):
        super().__init__('droidcam_publisher')

        self.declare_parameter('wifi', "P309")
        wifi = self.get_parameter('wifi').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Image, 'droidcam/image_raw', 10)
        self.timer = self.create_timer(0.020, self.timer_callback)  
        self.bridge = CvBridge()
        if wifi == "P309":
            stream_url = "http://10.42.0.232:4747/video"
        elif wifi == "MANH":
            stream_url ="http://192.168.196.148:4747/video"

        self.capture = cv2.VideoCapture(stream_url)

    def timer_callback(self):
        ret, frame = self.capture.read()
        if ret:
            # Convert the frame to a ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(ros_image)
            self.get_logger().info('Publishing image...')

def main():
    rclpy.init()
    droidcam_publisher = DroidCamPublisher()
    rclpy.spin(droidcam_publisher)

    droidcam_publisher.capture.release()
    cv2.destroyAllWindows()
    droidcam_publisher.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()