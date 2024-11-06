#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

# 全局变量，用于保存目标角度和距离
target_angle = 0.0
target_distance = float('inf')

def laser_callback(msg):
    global target_angle, target_distance
    max_diff = 3.0
    min_diff = 0.7
    max_index = 0

    # 遍历激光扫描数据，找到最大变化点作为目标点
    for i in range(1, len(msg.ranges) - 1):
        diff = abs(msg.ranges[i] - msg.ranges[i - 1])
        if (max_diff > diff > min_diff
            and msg.ranges[i] != 0.0
            and msg.ranges[i - 1] != 0.0
            and msg.ranges[i] < 1.5):
            max_index = i

    # 计算该点的角度和距离
    target_angle = msg.angle_min + max_index * msg.angle_increment
    target_distance = msg.ranges[max_index]

    rospy.loginfo("Target Point: Angle = {:.2f} degrees, Distance = {:.2f} meters".format(math.degrees(target_angle), target_distance))

def move_turtlebot():
    global target_angle, target_distance
    rospy.init_node('turtlebot_controller', anonymous=True)
    
    # 订阅激光扫描数据
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    
    # 创建发布者，发布到 /cmd_vel 话题
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # 设置发布频率
    rate = rospy.Rate(10)
    
    move_cmd = Twist()

    while not rospy.is_shutdown():
        # 检查目标距离，若在0.15米范围内则停止
        if target_distance < 0.2 and target_distance!=0:
            rospy.loginfo("Reached the target point")
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)
            break
        
        # 角度控制，角速度设置为目标角度的比例控制
        if target_angle > math.pi:
            target_angle = target_angle-2*math.pi
        move_cmd.angular.z = target_angle * 1.5  # 增加角速度以减少偏差

        # 距离控制，线速度与距离成比例
        move_cmd.linear.x = min(0.5, target_distance*0.5)  # 控制前进速度，使机器人逐渐接近目标

        # 发布速度命令
        pub.publish(move_cmd)
        rate.sleep()

    # 确保机器人停止
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        move_turtlebot()
    except rospy.ROSInterruptException:
        pass
