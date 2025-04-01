# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        print("Establishing QoS settings...")

        #match QoS to publisher
        qos_prof = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE,
            depth = 10
        )

        print("Initializing Teensy IMU 1 data subscription...")

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'imu1_data',
            self.imu_one_callback,
            qos_profile = qos_prof)
        self.subscription  # prevent unused variable warning

        print("Initializing Teensy GPS data subscription...")

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'gps_data',
            self.gps_callback,
            qos_profile = qos_prof)
        self.subscription  # prevent unused variable warning

        print("Initializing Teensy IMU 2 data subscription...")

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'imu2_data',
            self.imu_two_callback,
            qos_profile = qos_prof)
        self.subscription  # prevent unused variable warning

        print("Initializing Teensy IMU 3 data subscription...")

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'imu3_data',
            self.imu_three_callback,
            qos_profile = qos_prof)
        self.subscription  # prevent unused variable warning

        #print("Initializing console data publisher")
        
        
        self.imu1_publisher_ = self.create_publisher(
            Int32MultiArray,
            'console_imu1_data',
            qos_profile = qos_prof)

        self.imu2_publisher_ = self.create_publisher(
            Int32MultiArray,
            'console_imu2_data',
            qos_profile = qos_prof)

        self.imu3_publisher_ = self.create_publisher(
            Int32MultiArray,
            'console_imu3_data',
            qos_profile = qos_prof)

        self.gps_publisher_ = self.create_publisher(
            Float32MultiArray,
            'console_gps_data',
            qos_profile = qos_prof)
        
        


    def imu_one_callback(self, msg):
        self.get_logger().info('\nIMU 1 => ax: %s | ay: %s | az: %s | gx: %s | gy: %s | gz: %s' % (msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5]))
        self.imu1_publisher_.publish(msg)
        self.get_logger().info('Fowarding...')

    def imu_two_callback(self, msg):
        self.get_logger().info('\nIMU 2 => ax: %s | ay: %s | az: %s | gx: %s | gy: %s | gz: %s' % (msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5]))
        self.imu2_publisher_.publish(msg)
        self.get_logger().info('Fowarding...')

    def imu_three_callback(self, msg):
        self.get_logger().info('\nIMU 3 => ax: %s | ay: %s | az: %s | gx: %s | gy: %s | gz: %s' % (msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5]))
        self.imu3_publisher_.publish(msg)
        self.get_logger().info('Fowarding...')

    def gps_callback(self, msg):
        self.get_logger().info('\nGPS => Latitude: %s | Longitude: %s' % (msg.data[0], msg.data[1]))
        self.gps_publisher_.publish(msg)
        self.get_logger().info('Fowarding...')       


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
