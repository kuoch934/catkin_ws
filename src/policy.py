#!/usr/bin/env python3
# coding:utf-8

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from Policyfile.policy_runner import LoadedPolicy

RAD2DEG = 180.0 / np.pi
DEG2RAD = np.pi / 180.0


class PolicyNode:
    def __init__(self):
        # === 載入 NN ===
        self.policy = LoadedPolicy()
        rospy.loginfo(f"成功載入 policy: obs_dim={self.policy.obs_dim}, act_dim={self.policy.act_dim}")

        # === 參數 ===
        self.scale = rospy.get_param("~scale", 0.5)
        self.default_joint_pos = np.array(
            rospy.get_param("~default_joint_pos", [0.0] * self.policy.act_dim),
            dtype=np.float32
        )

        # === ROS topic ===
        self.pub = rospy.Publisher('/act', Float32MultiArray, queue_size=1)
        rospy.Subscriber('/cmd_vel', Float32MultiArray, self.cb_cmd_vel)
        rospy.Subscriber('/base_vel', Float32MultiArray, self.cb_base_vel)
        rospy.Subscriber('/proj_g', Float32MultiArray, self.cb_proj_g)
        rospy.Subscriber('/joint_pos', Float32MultiArray, self.cb_joint_pos)
        rospy.Subscriber('/joint_vel', Float32MultiArray, self.cb_joint_vel)

        # === 狀態初始化 ===
        self.cmd_vel   = np.zeros(3, dtype=np.float32)
        self.base_vel  = np.zeros(6, dtype=np.float32)
        self.proj_g    = np.zeros(3, dtype=np.float32)
        self.joint_pos = np.zeros(self.policy.act_dim, dtype=np.float32)
        self.joint_vel = np.zeros(self.policy.act_dim, dtype=np.float32)
        self.last_act  = np.zeros(self.policy.act_dim, dtype=np.float32)

        # === 頻率設定 ===
        self.step_count = 0
        self.output_div = 2                     # 每 2 次推論才發一次 (50 Hz → 25 Hz 傳輸)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.step)  # 50 Hz internal update

    # ===== Callbacks =====
    def cb_cmd_vel(self, msg): self.cmd_vel = np.array(msg.data, dtype=np.float32)
    def cb_base_vel(self, msg): self.base_vel = np.array(msg.data, dtype=np.float32)
    def cb_proj_g(self, msg): self.proj_g = np.array(msg.data, dtype=np.float32)
    def cb_joint_pos(self, msg): self.joint_pos = np.array(msg.data, dtype=np.float32) * DEG2RAD
    def cb_joint_vel(self, msg): self.joint_vel = np.array(msg.data, dtype=np.float32) * DEG2RAD

    # ===== Policy Step =====
    def step(self, event):
        self.step_count += 1

        # 構造觀測向量
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

        # NN 推論 (radians)
        raw_act = self.policy(obs)
        if raw_act.shape[0] != self.default_joint_pos.shape[0]:
            rospy.logwarn(f"Act 維度不符: policy={raw_act.shape[0]}, offset={self.default_joint_pos.shape[0]}")
            return

        act_rad = self.default_joint_pos + raw_act * self.scale
        self.last_act = act_rad.copy()

        # === 每 N 步才實際發送一次 (降低 UART 壓力) ===
        if self.step_count % self.output_div == 0:
            act_deg = act_rad * RAD2DEG
            self.pub.publish(Float32MultiArray(data=act_deg.tolist()))
            # rospy.loginfo_throttle(1.0, f"Publish /act @ {50/self.output_div} Hz")

if __name__ == '__main__':
    rospy.init_node('policy')
    node = PolicyNode()
    rospy.spin()


# #!/usr/bin/env python3
# # coding:utf-8

# import rospy
# import numpy as np
# from std_msgs.msg import Float32MultiArray
# from Policyfile.policy_runner import LoadedPolicy

# RAD2DEG = 180.0 / np.pi
# DEG2RAD = np.pi / 180.0

# # 現在所有關鍵輸入、輸出都已正確使用 float32。不需要再特別修改任何欄位。
# # 推論檔所需頻率為50hz

# # 9 KB/s × 10 bits = 90,000 bits/s ≈ 90 kbps
# # 這是 理論最低 baud rate（即 90,000 bps）。
# # 但實際上你還要預留 通訊開銷、ROS 封包包頭、rosserial 同步資訊，
# # 建議至少 ×2～×3 安全係數：

# # baud rate setting
# # 115200	可用，但接近上限（9 KB/s 約佔 80%）
# # 230400	穩定、安全
# # 460800	推薦，延遲極低、餘裕足夠
# # 921600	高速測試或未壓縮訊號時使用


# class PolicyNode:
#     def __init__(self):
#         # 載入 NN
#         self.policy = LoadedPolicy()
#         rospy.loginfo(f"成功載入 policy: obs_dim={self.policy.obs_dim}, act_dim={self.policy.act_dim}")
        
#         # === 控制參數 ===
#         self.scale = rospy.get_param("~scale", 0.5)
#         self.default_joint_pos = np.array(
#             rospy.get_param("~default_joint_pos", [0.0] * self.policy.act_dim),
#             dtype=np.float32
#         )

#         # Publisher
#         self.pub = rospy.Publisher('/act', Float32MultiArray, queue_size=1)

#         # Subscribers
#         rospy.Subscriber('/cmd_vel', Float32MultiArray, self.cb_cmd_vel)
#         rospy.Subscriber('/base_vel', Float32MultiArray, self.cb_base_vel)
#         rospy.Subscriber('/proj_g', Float32MultiArray, self.cb_proj_g)
#         rospy.Subscriber('/joint_pos', Float32MultiArray, self.cb_joint_pos)
#         rospy.Subscriber('/joint_vel', Float32MultiArray, self.cb_joint_vel)

#         # 初始化觀測
#         self.cmd_vel   = np.zeros(3, dtype=np.float32)
#         self.base_vel  = np.zeros(6, dtype=np.float32)
#         self.proj_g    = np.zeros(3, dtype=np.float32)
#         self.joint_pos = np.zeros(self.policy.act_dim, dtype=np.float32)
#         self.joint_vel = np.zeros(self.policy.act_dim, dtype=np.float32)
#         self.last_act  = np.zeros(self.policy.act_dim, dtype=np.float32)

#         # Timer
#         self.timer = rospy.Timer(rospy.Duration(0.02), self.step)  # 50Hz

#     # ===== Callbacks =====
#     def cb_cmd_vel(self, msg): self.cmd_vel = np.array(msg.data, dtype=np.float32)
#     def cb_base_vel(self, msg): self.base_vel = np.array(msg.data, dtype=np.float32)
#     def cb_proj_g(self, msg): self.proj_g = np.array(msg.data, dtype=np.float32)
#     def cb_joint_pos(self, msg): self.joint_pos = np.array(msg.data, dtype=np.float32) * DEG2RAD
#     def cb_joint_vel(self, msg): self.joint_vel = np.array(msg.data, dtype=np.float32) * DEG2RAD

#     # ===== Policy Step =====
#     def step(self, event):
#         obs = np.concatenate([
#             self.cmd_vel,
#             self.base_vel,
#             self.proj_g,
#             self.joint_pos,
#             self.joint_vel,
#             self.last_act
#         ])
#         if obs.shape[0] != self.policy.obs_dim:
#             rospy.logwarn(f"Obs 維度不符: policy={self.policy.obs_dim}, got {obs.shape[0]}")
#             return

#         # NN 推論 (radians)
#         raw_act = self.policy(obs)
#         if raw_act.shape[0] != self.default_joint_pos.shape[0]:
#             rospy.logwarn(f"Act 維度不符: policy={raw_act.shape[0]}, offset={self.default_joint_pos.shape[0]}")
#             return
        
#         act_rad = self.default_joint_pos + raw_act * self.scale
#         self.last_act = act_rad.copy()

#         # 發布前轉成 degree
#         act_deg = act_rad * RAD2DEG
#         self.pub.publish(Float32MultiArray(data=act_deg.tolist()))

# if __name__ == '__main__':
#     rospy.init_node('policy')
#     node = PolicyNode()
#     rospy.spin()
