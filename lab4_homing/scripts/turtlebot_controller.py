#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_turtlebot():
    # 初始化节点
    rospy.init_node('turtlebot_controller', anonymous=True)
    # 创建发布者，发布到 /cmd_vel 话题
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # 设置发布频率
    rate = rospy.Rate(10)

    # 创建Twist消息对象
    move_cmd = Twist()

    # 设置线速度和角速度
    move_cmd.linear.x = 0.01  # 前进速度
    move_cmd.angular.z = 0.0  # 不转弯

    # 发布速度信息
    while not rospy.is_shutdown():
        rospy.loginfo("Moving the TurtleBot")
        pub.publish(move_cmd)
        rate.sleep()

    # 停止机器人
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        move_turtlebot()
    except rospy.ROSInterruptException:
        pass
