#!/usr/bin/env python3
# coding:utf-8

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

from Policyfile.policy_runner import LoadedPolicy

class PolicyNode:
    def __init__(self):
        # 載入 NN
        self.policy = LoadedPolicy()
        rospy.loginfo(f"成功載入 policy: obs_dim={self.policy.obs_dim}, act_dim={self.policy.act_dim}")

        # Publisher
        self.pub = rospy.Publisher('/act', Float32MultiArray, queue_size=1)

        # Subscribers (依你訓練用的 obs 組合決定)
        rospy.Subscriber('/base_vel', Float32MultiArray, self.cb_base_vel)  # 例: 6維
        rospy.Subscriber('/proj_g', Float32MultiArray, self.cb_proj_g)      # 例: 3維
        rospy.Subscriber('/joint_states', JointState, self.cb_joint)        # 例: 位置+速度

        # 初始化各段 obs
        self.base_vel  = np.zeros(6, dtype=np.float32)
        self.proj_g    = np.zeros(3, dtype=np.float32)
        self.joint_pos = np.zeros(self.policy.act_dim, dtype=np.float32)
        self.joint_vel = np.zeros(self.policy.act_dim, dtype=np.float32)
        self.last_act  = np.zeros(self.policy.act_dim, dtype=np.float32)

        # Timer 以固定頻率組 obs 並推論
        self.timer = rospy.Timer(rospy.Duration(0.02), self.step)  # 50Hz

    # ===== Callbacks =====
    def cb_base_vel(self, msg): 
        self.base_vel = np.array(msg.data, dtype=np.float32)

    def cb_proj_g(self, msg):   
        self.proj_g   = np.array(msg.data, dtype=np.float32)
        
    def cb_joint(self, msg):
        self.joint_pos = np.array(msg.position, dtype=np.float32)
        self.joint_vel = np.array(msg.velocity, dtype=np.float32)

    # ===== Policy Step =====
    def step(self, event):
        # 把各段資料組成 obs
        obs = np.concatenate([
            self.base_vel,
            self.proj_g,
            self.joint_pos,
            self.joint_vel,
            self.last_act
        ])
        if obs.shape[0] != self.policy.obs_dim:
            rospy.logwarn(f"Obs 維度不符: got {obs.shape[0]} vs policy {self.policy.obs_dim}")
            return

        # NN 推論
        act = self.policy(obs)
        # 記住這次 act
        self.last_act = act.copy()
        # 發佈動作
        self.pub.publish(Float32MultiArray(data=act.tolist()))

if __name__ == '__main__':
    rospy.init_node('policy')
    node = PolicyNode()
    rospy.spin()
