#!/usr/bin/env python3
import os
import datetime
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs_py import point_cloud2

class ScanToPCNode(Node):
    def __init__(self):
        super().__init__('scan2pc')

        # 폴더 생성 (이미 존재하는 경우 생성하지 않음)
        if not os.path.exists('pcd'):
            os.makedirs('pcd')

        self.rpm = 12
        self.step_time = 60/self.rpm/2
        self.sx = 0
        self.sy = 0
        self.yaw = 0
        self.rotation_angle = 0  # 라이더 센서의 회전 각도
        self.rotation_speed = math.radians(50)  # 초당 회전 속도 (라디안 단위)
        self.previous_time = self.get_clock().now().to_msg()

        self.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]

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
        self.reverse = -1

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

        points = []
        for a in range(len(data.ranges)):
            if not np.isfinite(data.ranges[a]):
                continue  # 유효하지 않은 값은 건너뜁니다.

            ang = data.angle_increment * a + data.angle_min
            tmp_x = (data.ranges[a] * np.cos(ang))
            tmp_y = (data.ranges[a] * np.sin(ang))
            rotated_x = np.cos(self.rotation_angle) * tmp_x - np.sin(self.rotation_angle) * tmp_y
            rotated_y = np.sin(self.rotation_angle) * tmp_x + np.cos(self.rotation_angle) * tmp_y

            if not (np.isfinite(rotated_x) and np.isfinite(rotated_y)):
                continue  # 여기에서도 유효하지 않은 값은 건너뜁니다.

            x = self.sx + rotated_x
            y = self.sy + rotated_y
            z = 0
            points.append([x, y, z])
        
        ###############
        # check first #
        ###############
        pass

        ###############
        # check peaks #
        ###############
        pass

        ###############
        #  deal peaks #
        ###############
        if self.reverse == -1:
            self.pointq.append(points)
        elif self.reverse == 0:
            self.pointq.pop()
            self.pointq.

        # 포인트 클라우드 데이터를 PCD 파일로 저장

        header = Header()
        header.frame_id = "laser_link"
        pc2 = point_cloud2.create_cloud(header, self.fields, points)
        pc2.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(pc2)

        if True:
            self.save_to_pcd(points)

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
