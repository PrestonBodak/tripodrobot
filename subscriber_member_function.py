import threading
import tkinter
from tkinter import *

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ GUI CODE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#fill GUI labels with dummy data
def fill(root, data):
    labels = []

    for i in range(6):
        x = Label(root, text="{}: {}".format(pos[i], data[i]))
        x.pack()
        labels.append(x)
    
    return labels

tk = tkinter.Tk()
tk.title("Tripod Robot Control & Monitoring System")
tk.geometry("600x250")

#starting dummy data for GUI
pos = ["ax", "ay", "az", "gx", "gy", "gz"]
imu1_data = [1, 2, 3, 4, 5, 6]
imu2_data = [7, 8, 9, 10, 11, 12]
imu3_data = [13, 14, 15, 16, 17, 18]
gps_data = [12.5312, -51.6272]

monitor = Frame(tk)
monitor.pack()

#Leg 1 IMU
imu1 = LabelFrame(monitor, text="Leg 1 IMU")
imu1.grid(row = 0, column = 0)
imu1_labels = fill(imu1, imu1_data)

#Leg 2 IMU
imu2 = LabelFrame(monitor, text="Leg 2 IMU")
imu2.grid(row = 0, column = 1)
imu2_labels = fill(imu2, imu2_data)

#Leg 3 IMU
imu3 = LabelFrame(monitor, text="Leg 3 IMU")
imu3.grid(row = 0, column = 2)
imu3_labels = fill(imu3, imu3_data)

#GPS
gps = LabelFrame(monitor, text="GPS")
gps.grid(row = 0, column = 3)
lat = Label(gps, text="Latitude: {}".format(gps_data[0]))
long = Label(gps, text="Longitude: {}".format(gps_data[1]))
lat.pack()
long.pack()

control = Frame(tk)
control.pack()

#Start/Stop
start = Button(control, text="Start")

def starttoggle():
    #TODO: Publish to start topic here

    if start["text"] == "Start":
        start.config(text="Stop")
    else:
        start.config(text="Start")

start.config(command=starttoggle)
start.pack(side=LEFT)

#Turn?
turn = Button(control, text="Turn")
turn.pack(side=RIGHT)







#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ROS2 CODE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        print("Initializing...")

        #match quality of service with data publisher
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
        #update IMU labels with received data
        for i in range(6):
            imu1_labels[i].config(text="{}: {}".format(pos[i], msg.data[i]))
    
    def imu2_listener_callback(self, msg):
        self.get_logger().info('IMU2 => I heard: "%s"' % msg.data[0])
        #update IMU labels
        for i in range(6):
            imu2_labels[i].config(text="{}: {}".format(pos[i], msg.data[i]))

    def imu3_listener_callback(self, msg):
        self.get_logger().info('IMU3 => I heard: "%s"' % msg.data[0])
        #update IMU labels
        for i in range(6):
            imu3_labels[i].config(text="{}: {}".format(pos[i], msg.data[i]))

    def gps_listener_callback(self, msg):
        self.get_logger().info('GPS => I heard: "%s"' % msg.data[0])
        #update GPS label
        lat.config(text="Latitude: {:.4f}".format(msg.data[0]))
        long.config(text="Longitude: {:.4f}".format(msg.data[1]))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    print("Spinning node...")

    #start ROS2 in separate thread (rclpy.spin here would prevent the GUI from loading)
    thread_spin = threading.Thread(target=rclpy.spin, args=(minimal_subscriber, ))
    thread_spin.start()

    #start the GUI
    tk.mainloop()


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
