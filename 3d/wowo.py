#!/usr/bin/python
import os
import datetime
import roslib
import rospy
import tf
import math
import numpy as np

from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# 폴더 생성 (이미 존재하는 경우 생성하지 않음)
if not os.path.exists('pcd'):
    os.makedirs('pcd')

def save_to_pcd(points, filename):
    with open(filename, 'w') as f:
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

# ROS 노드 초기화
rospy.init_node('scan2pc')

sx = 0
sy = 0
yaw = 0
rotation_angle = 0  # 라이더 센서의 회전 각도
rotation_speed = math.radians(50)  # 초당 회전 속도 (라디안 단위)
previous_time = rospy.Time.now().to_sec()

header = Header()
fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1)]

def SlamOdometryData(data):
    global sx, sy, yaw
    sx = data.pose.position.x
    sy = data.pose.position.y
    orientation_list = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

def processLaserScan(data):
    global sx, sy, header, fields, points, yaw, rotation_angle, previous_time, rotation_speed
    current_time = rospy.Time.now().to_sec()
    rotation_angle += rotation_speed * (current_time - previous_time)
    previous_time = current_time

    points = []
    for a in range(len(data.ranges)):
        if not np.isfinite(data.ranges[a]):
            continue  # 유효하지 않은 값은 건너뜁니다.

        ang = data.angle_increment * a + data.angle_min
        tmp_x = (data.ranges[a] * np.cos(ang))
        tmp_y = (data.ranges[a] * np.sin(ang))
        rotated_x = np.cos(rotation_angle) * tmp_x - np.sin(rotation_angle) * tmp_y
        rotated_y = np.sin(rotation_angle) * tmp_x + np.cos(rotation_angle) * tmp_y

        if not (np.isfinite(rotated_x) and np.isfinite(rotated_y)):
            continue  # 여기에서도 유효하지 않은 값은 건너뜁니다.

        x = sx + rotated_x
        y = sy + rotated_y
        z = 0
        points.append([x, y, z])

    # 포인트 클라우드 데이터를 PCD 파일로 저장
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    pcd_filename = f"pcd/scan_{timestamp}.pcd"
    save_to_pcd(points, pcd_filename)

    pc2 = point_cloud2.create_cloud(header, fields, points)
    pc2.header.stamp = rospy.Time.now()
    pubCloud.publish(pc2)

rospy.Subscriber('/scan', LaserScan, processLaserScan)
pubCloud = rospy.Publisher('/laser_cloud_surround_hector', PointCloud2, queue_size=10)
header.frame_id = "map"

while not rospy.is_shutdown():    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        break

