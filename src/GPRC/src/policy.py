#!/usr/bin/env python3
# coding:utf-8

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

from Policyfile.policy_runner import LoadedPolicy

class PolicyNode:
    def __init__(self):
        # 載入 NN
        self.policy = LoadedPolicy()
        rospy.loginfo(f"成功載入 policy: obs_dim={self.policy.obs_dim}, act_dim={self.policy.act_dim}")
        
        # === 控制參數 ===
        # 動作縮放因子，可以用 rosparam 覆蓋
        self.scale = rospy.get_param("~scale", 0.5)  

        # 關節初始偏移 (站立姿勢)
        self.default_joint_pos = np.array(
            rospy.get_param("~default_joint_pos", [0.0] * self.policy.act_dim),
            dtype=np.float32
        )

        # Publisher
        self.pub = rospy.Publisher('/act', Float32MultiArray, queue_size=1)

        # Subscribers (依你訓練用的 obs 組合決定)
        rospy.Subscriber('/cmd_vel', Twist, self.cb_cmd_vel)
        rospy.Subscriber('/base_vel', Twist, self.cb_base_vel)  # 例: 6維
        rospy.Subscriber('/proj_g', Float32MultiArray, self.cb_proj_g)      # 例: 3維
        rospy.Subscriber('/joint_states', JointState, self.cb_joint)        # 例: 位置+速度

        # 初始化各段 obs
        self.cmd_vel   = np.zeros(3, dtype=np.float32)  # 例: 3維 (例: vx, vy, wz)
        self.base_vel  = np.zeros(6, dtype=np.float32)  # 例: linear(3) + angular(3)
        self.proj_g    = np.zeros(3, dtype=np.float32)  # 投影重力向量
        self.joint_pos = np.zeros(self.policy.act_dim, dtype=np.float32)
        self.joint_vel = np.zeros(self.policy.act_dim, dtype=np.float32)
        self.last_act  = np.zeros(self.policy.act_dim, dtype=np.float32)

        # Timer 以固定頻率組 obs 並推論(一定要放在最後) step 在這裡被使用
        self.timer = rospy.Timer(rospy.Duration(0.02), self.step)  # 50Hz 

    # ===== Callbacks =====
    def cb_cmd_vel(self, msg: Twist):
        # 平面 2D: vx, vy, yaw_rate
        self.cmd_vel = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.angular.z
        ], dtype=np.float32)
    def cb_base_vel(self, msg: Twist): 
        self.base_vel = np.array([
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z
        ], dtype=np.float32)

    def cb_proj_g(self, msg):   
        self.proj_g   = np.array(msg.data, dtype=np.float32)
        
    def cb_joint(self, msg):
        self.joint_pos = np.array(msg.position, dtype=np.float32)
        self.joint_vel = np.array(msg.velocity, dtype=np.float32)

    # ===== Policy Step =====
    def step(self, event):
        # 把各段資料組成 obs
        obs = np.concatenate([
            self.cmd_vel,
            self.base_vel,
            self.proj_g,
            self.joint_pos,
            self.joint_vel,
            self.last_act
        ])
        if obs.shape[0] != self.policy.obs_dim:
            rospy.logwarn(f"Obs 維度不符: policy={self.policy.obs_dim}, got {obs.shape[0]}")
            return

        # NN 推論
        raw_act = self.policy(obs)  
        if raw_act.shape[0] != self.default_joint_pos.shape[0]:
            rospy.logwarn(f"Act 維度不符: policy={raw_act.shape[0]}, offset={self.default_joint_pos.shape[0]}")
            return   
        act = self.default_joint_pos + raw_act * self.scale # 縮放 + 偏移
        self.last_act = act.copy() # 更新 last_act
        # 發佈動作
        self.pub.publish(Float32MultiArray(data=act.tolist()))

if __name__ == '__main__':
    rospy.init_node('policy')
    node = PolicyNode()
    rospy.spin()
