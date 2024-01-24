#!/usr/bin/env python3
import os
import datetime
import math
import numpy as np
import rclpy
from time import time
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs_py import point_cloud2
import serial

class ScanToPCNode(Node):
    def __init__(self):
        super().__init__('scan2pc')


        #PARAMETERS###########################################################
        self.save = True                                                     #
        self.rpm = 2                                                         #
        self.hz = 10                                                         #
        self.port = '/dev/ttyACM0'                                           #
        self.baudrate = 9600                                                 #
        ######################################################################


        self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
        self.rev_time = 30/self.rpm
        self.rev_size = 30/self.rpm*self.hz
        self.yaw_step = np.pi/self.rev_size
        self.index = 0
        self.yaw = 0
        self.init = False
        self.rotation_angle = 0  # 라이더 센서의 회전 각도
        self.rotation_speed = math.radians(50)  # 초당 회전 속도 (라디안 단위)
        self.previous_time = self.get_clock().now().to_msg()
        self.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        #self.time = time()
        self.points3d = []


        # 폴더 생성 (이미 존재하는 경우 생성하지 않음)
        if not os.path.exists('pcd'):
            os.makedirs('pcd')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.process_laser_scan,
            10)
        self.publisher = self.create_publisher(
            PointCloud2,
            '/laser_cloud_surround_hector',
            10)
        self.pointq = []
        self.reverse = None
        self.flip = False

        self.get_logger().info('Node initialized')

    def save_to_pcd(self, points):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        pcd_filename = f"pcd/scan_{timestamp}.pcd"
        with open(pcd_filename, 'w') as f:
            f.write("VERSION .7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write("WIDTH {}\n".format(len(points)))
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write("POINTS {}\n".format(len(points)))
            f.write("DATA ascii\n")
            for point in points:
                f.write("{} {} {}\n".format(point[0], point[1], point[2]))

    def process_laser_scan(self, data):
        current_time = self.get_clock().now().to_msg()
        self.rotation_angle += self.rotation_speed * (current_time.nanosec - self.previous_time.nanosec)
        self.previous_time = current_time

        if not self.init:
            self.ang = np.array(list(range(len(data.ranges))))
            self.ang = self.ang * data.angle_increment + data.angle_min
            self.xmul = np.cos(self.ang)
            self.ymul = np.sin(self.ang)
            self.init = True
        arr = np.array(data.ranges)
        x = arr*self.xmul
        y = arr*self.ymul
        z = np.array([0]*len(data.ranges))
        # rotate 90deg y-axis: [x,y,z] -> [z,y,-x]
        arr = np.transpose(np.array([z,y,-x]))

        self.check_rotation()
        # rotate z-axis
        if self.flip:
            if self.save and not self.reverse==None: self.save_to_pcd(self.points3d)
            self.points3d = []
            self.flip = False
            if self.reverse:
                #print("ccw")
                self.yaw = np.pi
            else: 
                #print("cw")
                self.yaw = 0    
        rot = np.array([[np.cos(self.yaw), 0-np.sin(self.yaw), 0],
                        [np.sin(self.yaw), np.cos(self.yaw), 0],
                        [0,0,1]])
        #print(rot)
        arr = np.matmul(arr,rot)
        if self.reverse:
            self.yaw -= self.yaw_step
        else:
            self.yaw += self.yaw_step

        points = arr.tolist()
        self.points3d += points

        header = Header()
        header.frame_id = "laser"
        pc2 = point_cloud2.create_cloud(header, self.fields, points)
        pc2.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(pc2)

    def check_rotation(self):
        if self.serial.in_waiting > 0:
            line = self.serial.readline().decode('utf-8').rstrip()
            print(line)
            self.flip = True
            if line == "ccw": self.reverse = True
            else: self.reverse = False

def main(args=None):
    rclpy.init(args=args)
    scan_to_pc_node = ScanToPCNode()

    try:
        rclpy.spin(scan_to_pc_node)
    except KeyboardInterrupt:
        pass

    scan_to_pc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
