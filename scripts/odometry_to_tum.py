#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from nav_msgs.msg import Odometry

class OdomRecorder:
    def __init__(self, topic_name, output_txt):
        self.output_txt = output_txt
        # 打开文件准备写入
        try:
            self.file = open(self.output_txt, 'w')
        except Exception as e:
            rospy.logerr(f"无法打开或创建文件: {e}")
            sys.exit(1)
            
        self.count = 0
        
        # 注册节点关闭时的回调，确保文件安全关闭
        rospy.on_shutdown(self.shutdown_hook)
        
        # 订阅指定的 Odometry 话题
        self.sub = rospy.Subscriber(topic_name, Odometry, self.odom_callback)
        rospy.loginfo(f"开始监听话题: {topic_name}")
        rospy.loginfo(f"数据将实时保存至: {self.output_txt}")
        rospy.loginfo("按 Ctrl+C 停止记录...")

    def odom_callback(self, msg):
        # 提取时间戳 (转为秒)
        timestamp = msg.header.stamp.to_sec()
        
        # 提取平移 (Translation)
        tx = msg.pose.pose.position.x
        ty = msg.pose.pose.position.y
        tz = msg.pose.pose.position.z
        
        # 提取旋转四元数 (Quaternion)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # 写入 TUM 格式: timestamp x y z qx qy qz qw
        self.file.write(f"{timestamp:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
        
        self.count += 1
        # 每记录 100 帧打印一次提示，证明程序活着
        if self.count % 100 == 0:
            rospy.loginfo(f"已记录 {self.count} 帧位姿...")

    def shutdown_hook(self):
        # 节点关闭时，保存并关闭文件
        if not self.file.closed:
            self.file.close()
            rospy.loginfo(f"记录结束！共保存了 {self.count} 帧位姿。")
            rospy.loginfo(f"文件位置: {self.output_txt}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("使用方法: python odometry_to_tum.py <Odometry话题名> <输出的txt路径>")
        print("示例: python odometry_to_tum.py /vio/odom trajectory.txt")
        sys.exit(1)
        
    topic_name = sys.argv[1]
    output_txt = sys.argv[2]
    
    # 初始化 ROS 节点
    rospy.init_node('realtime_odom_recorder', anonymous=True)
    
    # 实例化记录器
    recorder = OdomRecorder(topic_name, output_txt)
    
    # 保持节点运行，直到按 Ctrl+C
    rospy.spin()