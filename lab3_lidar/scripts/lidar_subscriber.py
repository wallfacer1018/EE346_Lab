#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import csv

def lidar_callback(data):
    # 输出最小和最大测距范围
    rospy.loginfo("Minimum Range: %f, Maximum Range: %f", data.range_min, data.range_max)

    # 打印前方（0度）方向的测距数据
    front_range = data.ranges[int(len(data.ranges)/2)]
    rospy.loginfo("Front Distance: %f", front_range)

    # 打印全部范围的测距数据
    for i, range_val in enumerate(data.ranges):
        rospy.loginfo("Angle: %f, Distance: %f", data.angle_min + i * data.angle_increment, range_val)
    
    with open('/home/wby/catkin_ws/Lab3/src/ladar_data.csv', 'a') as f:
        writer = csv.writer(f)
        # 记录角度和距离数据
        for i, range_val in enumerate(data.ranges):
            angle = data.angle_min + i * data.angle_increment
            writer.writerow([angle, range_val])

    rospy.loginfo("Data saved to CSV file")

def lidar_listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.spin()

if __name__ == '__main__':
    lidar_listener()
