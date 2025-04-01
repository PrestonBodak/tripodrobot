import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        print("Initializing...")

        qos_prof = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE,
            depth = 10
        )

        print("Creating data subscriptions...")

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'console_imu1_data',
            self.imu1_listener_callback,
            qos_profile = qos_prof)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'console_imu2_data',
            self.imu2_listener_callback,
            qos_profile = qos_prof)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'console_imu3_data',
            self.imu3_listener_callback,
            qos_profile = qos_prof)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'console_gps_data',
            self.gps_listener_callback,
            qos_profile = qos_prof)
        self.subscription  # prevent unused variable warning

        print("Data subscription created!")

    def imu1_listener_callback(self, msg):
        self.get_logger().info('IMU1 => I heard: "%s"' % msg.data[0])
    
    def imu2_listener_callback(self, msg):
        self.get_logger().info('IMU2 => I heard: "%s"' % msg.data[0])

    def imu3_listener_callback(self, msg):
        self.get_logger().info('IMU3 => I heard: "%s"' % msg.data[0])

    def gps_listener_callback(self, msg):
        self.get_logger().info('GPS => I heard: "%s"' % msg.data[0])


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    print("Spinning node...")

    rclpy.spin(minimal_subscriber)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
