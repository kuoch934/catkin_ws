#!/usr/bin/env python3
# coding:utf-8
"""
policy_tester.py
模擬假資料輸入給 policy_node，檢查整體 pipeline 是否正常
"""
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

def main():
    rospy.init_node('policy_tester')
    rospy.loginfo("✅ policy_tester 啟動，開始發送假資料...")

    # Publisher
    pub_cmd  = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_base = rospy.Publisher('/base_vel', Twist, queue_size=1)
    pub_proj = rospy.Publisher('/proj_g', Float32MultiArray, queue_size=1)
    pub_joint = rospy.Publisher('/joint_states', JointState, queue_size=1)

    # 測試參數
    rate = rospy.Rate(50)  # 50Hz
    t = 0.0
    n_joints = rospy.get_param("~n_joints", 12)  # 根據 policy.act_dim 設定

    while not rospy.is_shutdown():
        # === 模擬移動指令 ===
        cmd = Twist()
        cmd.linear.x = 0.2 * np.sin(t)
        cmd.linear.y = 0.1 * np.cos(t)
        cmd.angular.z = 0.3 * np.sin(0.5 * t)
        pub_cmd.publish(cmd)

        # === 模擬 base velocity ===
        base = Twist()
        base.linear.x = 0.2 * np.sin(t)
        base.linear.y = 0.1 * np.cos(t)
        base.linear.z = 0.0
        base.angular.x = 0.0
        base.angular.y = 0.0
        base.angular.z = 0.3 * np.sin(0.5 * t)
        pub_base.publish(base)

        # === 模擬投影重力向量 ===
        pub_proj.publish(Float32MultiArray(data=[0.0, 0.0, -1.0]))

        # === 模擬關節狀態 ===
        js = JointState()
        js.position = 0.1 * np.sin(np.linspace(0, np.pi, n_joints) + t)
        js.velocity = 0.05 * np.cos(np.linspace(0, np.pi, n_joints) + t)
        pub_joint.publish(js)

        # 時間累積
        t += 0.02
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
