#!/usr/bin/env python
# coding:utf-8
import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64MultiArray
import numpy as np

class TransformPoseNode:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('transform_pose_node')

        # 创建 tf2 变换监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 订阅 PoseStamped 主题
        self.subscriber1 = rospy.Subscriber('/object1', PoseStamped, self.pose_callback)
        self.subscriber2 = rospy.Subscriber('/object2', PoseStamped, self.pose_callback)
        self.subscriber3 = rospy.Subscriber('/object3', PoseStamped, self.pose_callback)

        # 发布转换后的 Pose
        self.publisher = rospy.Publisher('/transformed_pose', PoseStamped, queue_size=10)

    def pose_callback(self, msg):
        try:
            # 将 PoseStamped 转换到全局坐标系 (例如 'map')
            transformed_pose = self.transform_pose(msg, 'map')
            # 发布转换后的 Pose
            self.publisher.publish(transformed_pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF error: {e}")

    def transform_pose(self, pose_stamped, target_frame):
        # 获取当前时间戳
        current_time = rospy.Time.now()
        # 转换 PoseStamped 到目标坐标系
        transformed_pose = self.tf_buffer.transform(pose_stamped, target_frame, current_time)
        return transformed_pose

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = TransformPoseNode()
        node.run()
    except rospy.ROSInterruptException:
        pass