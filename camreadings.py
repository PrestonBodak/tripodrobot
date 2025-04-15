import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2

class CamSubscriber(Node):
    def __init__(self):
        super().__init__("intel_subscriber")

        # Create subscriptions for RGB and Object Detection topics from three cameras
        self.subscription_rgb_1 = self.create_subscription(
            Image, "camera_1/rgb_frame", self.rgb_frame_callback_1, 10
        )
        self.subscription_detections_1 = self.create_subscription(
            Detection2DArray, "camera_1/detections", self.detection_callback_1, 10
        )

        self.subscription_rgb_2 = self.create_subscription(
            Image, "camera_2/rgb_frame", self.rgb_frame_callback_2, 10
        )
        self.subscription_detections_2 = self.create_subscription(
            Detection2DArray, "camera_2/detections", self.detection_callback_2, 10
        )

        self.subscription_rgb_3 = self.create_subscription(
            Image, "camera_3/rgb_frame", self.rgb_frame_callback_3, 10
        )
        self.subscription_detections_3 = self.create_subscription(
            Detection2DArray, "camera_3/detections", self.detection_callback_3, 10
        )

        # Initialize CvBridge to convert ROS images to OpenCV images
        self.br = CvBridge()

        # Variables to hold frames and detections
        self.current_rgb_1 = None
        self.current_detections_1 = []

        self.current_rgb_2 = None
        self.current_detections_2 = []

        self.current_rgb_3 = None
        self.current_detections_3 = []

        # Set up a timer to regularly call the display function
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 FPS, adjust if necessary

    # RGB frame callbacks for Camera 1
    def rgb_frame_callback_1(self, msg):
        self.get_logger().info("Receiving Camera 1 RGB frame")
        self.current_rgb_1 = self.br.imgmsg_to_cv2(msg)

    def detection_callback_1(self, detections_msg):
        self.get_logger().info("Receiving Camera 1 Object Detection data")
        self.current_detections_1 = detections_msg.detections

    # RGB frame callbacks for Camera 2
    def rgb_frame_callback_2(self, msg):
        self.get_logger().info("Receiving Camera 2 RGB frame")
        self.current_rgb_2 = self.br.imgmsg_to_cv2(msg)

    def detection_callback_2(self, detections_msg):
        self.get_logger().info("Receiving Camera 2 Object Detection data")
        self.current_detections_2 = detections_msg.detections

    # RGB frame callbacks for Camera 3
    def rgb_frame_callback_3(self, msg):
        self.get_logger().info("Receiving Camera 3 RGB frame")
        self.current_rgb_3 = self.br.imgmsg_to_cv2(msg)

    def detection_callback_3(self, detections_msg):
        self.get_logger().info("Receiving Camera 3 Object Detection data")
        self.current_detections_3 = detections_msg.detections

    def display_data(self):
        """ Display the frames and detections for each camera """
        if self.current_rgb_1 is not None:
            for detection in self.current_detections_1:
                x = int(detection.bbox.center.position.x)
                y = int(detection.bbox.center.position.y)
                w = int(detection.bbox.size_x)
                h = int(detection.bbox.size_y)
                cv2.rectangle(self.current_rgb_1, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(self.current_rgb_1, f"{x},{y}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("Camera 1 RGB with Detection", self.current_rgb_1)

        if self.current_rgb_2 is not None:
            for detection in self.current_detections_2:
                x = int(detection.bbox.center.position.x)
                y = int(detection.bbox.center.position.y)
                w = int(detection.bbox.size_x)
                h = int(detection.bbox.size_y)
                cv2.rectangle(self.current_rgb_2, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(self.current_rgb_2, f"{x},{y}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("Camera 2 RGB with Detection", self.current_rgb_2)

        if self.current_rgb_3 is not None:
            for detection in self.current_detections_3:
                x = int(detection.bbox.center.position.x)
                y = int(detection.bbox.center.position.y)
                w = int(detection.bbox.size_x)
                h = int(detection.bbox.size_y)
                cv2.rectangle(self.current_rgb_3, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(self.current_rgb_3, f"{x},{y}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("Camera 3 RGB with Detection", self.current_rgb_3)

        cv2.waitKey(1)

    def timer_callback(self):
        """ Regular callback to update the frames every time step """
        self.display_data()


def main(args=None):
    rclpy.init(args=args)
    cam_subscriber = CamSubscriber()
    rclpy.spin(cam_subscriber)
    cam_subscriber.destroy_node()
    rclpy.shutdown()

    # Close all OpenCV windows when shutting down
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
