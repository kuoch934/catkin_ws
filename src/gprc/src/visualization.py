#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64MultiArray

class HumanoidLegVisualizer:
    def __init__(self):
        rospy.init_node('humanoid_leg_visualizer', anonymous=True)
        
        # ========== MODE SELECTION ==========
        # Set to True for test mode (use test_angles below)
        # Set to False for ROS topic mode (subscribe to /act)
        self.TEST_MODE = False
        # ====================================
        
        # Joint angle data (12 DOF in the order provided)
        self.joint_angles = np.zeros(12)
        
        # ========== TEST ANGLES (for testing joint directions) ==========
        # Modify these values to test different poses
        # All angles in DEGREES (will be converted to radians internally)
        self.test_angles = np.array([
            0.0,    # left_hip_yaw
            0.0,    # right_hip_yaw
            0.0,    # left_hip_roll
            0.0,    # right_hip_roll
            0.0,    # left_hip_pitch
            0.0,    # right_hip_pitch
            0.0,    # left_knee
            0.0,    # right_knee
            0.0,    # left_ankle_roll
            0.0,    # right_ankle_roll
            0.0,    # left_ankle_pitch
            0.0     # right_ankle_pitch
        ])
        
        # Example test poses (uncomment to use):
        # Standing pose
        # self.test_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Left leg raised
        # self.test_angles = np.array([0.0, 0.0, 0.0, 0.0, 28.6, 0.0, 45.8, 0.0, 0.0, 0.0, -17.2, 0.0])
        
        # Both knees bent
        self.test_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 57.3, 57.3, 0.0, 0.0, 0.0, 0.0])
        
        # Walking pose
        # self.test_angles = np.array([5.7, -5.7, 8.6, -8.6, 22.9, -11.5, 40.1, 17.2, 2.9, -2.9, -8.6, 5.7])
        # ================================================================
        
        # Joint name mapping
        self.joint_names = [
            'left_hip_yaw', 'right_hip_yaw',
            'left_hip_roll', 'right_hip_roll',
            'left_hip_pitch', 'right_hip_pitch',
            'left_knee', 'right_knee',
            'left_ankle_roll', 'right_ankle_roll',
            'left_ankle_pitch', 'right_ankle_pitch'
        ]
        
        # Joint index mapping
        self.LEFT_HIP_YAW = 0
        self.RIGHT_HIP_YAW = 1
        self.LEFT_HIP_ROLL = 2
        self.RIGHT_HIP_ROLL = 3
        self.LEFT_HIP_PITCH = 4
        self.RIGHT_HIP_PITCH = 5
        self.LEFT_KNEE = 6
        self.RIGHT_KNEE = 7
        self.LEFT_ANKLE_ROLL = 8
        self.RIGHT_ANKLE_ROLL = 9
        self.LEFT_ANKLE_PITCH = 10
        self.RIGHT_ANKLE_PITCH = 11
        
        # Robot dimensions (meters) - adjust to match your robot
        self.hip_width = 0.2        # Hip width
        self.thigh_length = 0.4     # Upper leg length
        self.shin_length = 0.4      # Lower leg length
        self.foot_length = 0.1      # Foot length
        self.torso_height = 0.1     # Torso to hip vertical distance
        
        # Create figure
        self.fig = plt.figure(figsize=(12, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.setup_plot()
        
        # Subscribe to /act topic only if not in test mode
        if not self.TEST_MODE:
            self.subscriber = rospy.Subscriber(
                '/act', 
                Float64MultiArray,
                self.joint_callback,
                queue_size=1
            )
            rospy.loginfo("Humanoid Leg Visualizer started in TOPIC MODE")
            rospy.loginfo("Subscribing to topic: /act")
        else:
            # Convert test angles from degrees to radians
            self.joint_angles = self.test_angles.copy() * np.pi / 180.0
            rospy.loginfo("Humanoid Leg Visualizer started in TEST MODE")
            rospy.loginfo("Using predefined test angles")
            rospy.loginfo(f"Test angles (deg): {self.test_angles}")
            rospy.loginfo(f"Test angles (rad): {self.joint_angles}")
    
    def setup_plot(self):
        """Setup 3D plot parameters"""
        self.ax.set_xlabel('X (m)', fontsize=10)
        self.ax.set_ylabel('Y (m)', fontsize=10)
        self.ax.set_zlabel('Z (m)', fontsize=10)
        
        mode_text = "TEST MODE" if self.TEST_MODE else "TOPIC MODE"
        self.ax.set_title(f'Humanoid Robot Leg Visualization - {mode_text}', 
                         fontsize=12, fontweight='bold')
        
        # Set display range
        limit = 0.6
        self.ax.set_xlim([-limit, limit])
        self.ax.set_ylim([-limit, limit])
        self.ax.set_zlim([0, 1.2])
        
        # Set viewing angle (elevation, azimuth)
        self.ax.view_init(elev=20, azim=45)
        
        # Add grid
        self.ax.grid(True, alpha=0.3)
    
    def joint_callback(self, msg):
        """Callback function to receive joint angles (in degrees from topic)"""
        if len(msg.data) == 12:
            # Convert from degrees to radians
            self.joint_angles = np.array(msg.data) * np.pi / 180.0
            rospy.logdebug(f"Received joint angles (deg): {msg.data}")
            rospy.logdebug(f"Converted to radians: {self.joint_angles}")
        else:
            rospy.logwarn(f"Invalid joint data length: {len(msg.data)}, expected: 12")
    
    def rotation_matrix_z(self, angle):
        """Rotation matrix around Z-axis (yaw)"""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [c, -s, 0],
            [s, c, 0],
            [0, 0, 1]
        ])
    
    def rotation_matrix_x(self, angle):
        """Rotation matrix around X-axis (roll)"""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [1, 0, 0],
            [0, c, -s],
            [0, s, c]
        ])
    
    def rotation_matrix_y(self, angle):
        """Rotation matrix around Y-axis (pitch)"""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [c, 0, s],
            [0, 1, 0],
            [-s, 0, c]
        ])
    
    def calculate_leg_chain(self, hip_pos, hip_yaw, hip_roll, hip_pitch, 
                           knee_pitch, ankle_roll, ankle_pitch, side='left'):
        """
        Calculate kinematic chain for one leg
        
        Parameters:
            hip_pos: Hip joint position
            hip_yaw, hip_roll, hip_pitch: Hip joint angles
            knee_pitch: Knee joint angle
            ankle_roll, ankle_pitch: Ankle joint angles
            side: 'left' or 'right'
        """
        positions = [hip_pos]
        
        # Calculate combined hip rotation matrix (order: yaw -> roll -> pitch)
        R_hip = self.rotation_matrix_z(hip_yaw) @ \
                self.rotation_matrix_x(hip_roll) @ \
                self.rotation_matrix_y(hip_pitch)
        
        # Thigh direction (initially pointing down)
        thigh_dir = R_hip @ np.array([0, 0, -1])
        knee_pos = hip_pos + thigh_dir * self.thigh_length
        positions.append(knee_pos)
        
        # Shin direction (rotate knee angle on top of hip rotation)
        R_knee = R_hip @ self.rotation_matrix_y(knee_pitch)
        shin_dir = R_knee @ np.array([0, 0, -1])
        ankle_pos = knee_pos + shin_dir * self.shin_length
        positions.append(ankle_pos)
        
        # Foot direction (add ankle rotation on top of shin)
        R_ankle = R_knee @ \
                  self.rotation_matrix_x(ankle_roll) @ \
                  self.rotation_matrix_y(ankle_pitch)
        foot_dir = R_ankle @ np.array([1, 0, 0])  # Foot points forward
        foot_pos = ankle_pos + foot_dir * self.foot_length
        positions.append(foot_pos)
        
        return np.array(positions)
    
    def forward_kinematics(self):
        """Calculate forward kinematics for the entire robot"""
        # Torso center (base)
        base_height = 0.9
        pelvis_center = np.array([0.0, 0.0, base_height])
        
        # Left and right hip positions
        left_hip_pos = pelvis_center + np.array([0, self.hip_width/2, -self.torso_height])
        right_hip_pos = pelvis_center + np.array([0, -self.hip_width/2, -self.torso_height])
        
        # Calculate left leg joint positions
        left_leg = self.calculate_leg_chain(
            left_hip_pos,
            self.joint_angles[self.LEFT_HIP_YAW],
            self.joint_angles[self.LEFT_HIP_ROLL],
            self.joint_angles[self.LEFT_HIP_PITCH],
            self.joint_angles[self.LEFT_KNEE],
            self.joint_angles[self.LEFT_ANKLE_ROLL],
            self.joint_angles[self.LEFT_ANKLE_PITCH],
            side='left'
        )
        
        # Calculate right leg joint positions
        right_leg = self.calculate_leg_chain(
            right_hip_pos,
            self.joint_angles[self.RIGHT_HIP_YAW],
            self.joint_angles[self.RIGHT_HIP_ROLL],
            self.joint_angles[self.RIGHT_HIP_PITCH],
            self.joint_angles[self.RIGHT_KNEE],
            self.joint_angles[self.RIGHT_ANKLE_ROLL],
            self.joint_angles[self.RIGHT_ANKLE_PITCH],
            side='right'
        )
        
        return {
            'pelvis': pelvis_center,
            'left_hip': left_hip_pos,
            'right_hip': right_hip_pos,
            'left_leg': left_leg,
            'right_leg': right_leg
        }
    
    def update_plot(self, frame):
        """Animation update function"""
        self.ax.clear()
        self.setup_plot()
        
        # In test mode, use static test angles (convert from degrees to radians)
        if self.TEST_MODE:
            self.joint_angles = self.test_angles.copy() * np.pi / 180.0
        
        # Calculate all joint positions
        kin = self.forward_kinematics()
        
        # Draw torso (hip connection)
        pelvis_points = np.array([
            kin['left_hip'],
            kin['pelvis'],
            kin['right_hip']
        ])
        self.ax.plot(pelvis_points[:, 0], pelvis_points[:, 1], pelvis_points[:, 2],
                    'k-', linewidth=4, label='Torso')
        
        # Draw left leg
        left_leg = kin['left_leg']
        self.ax.plot(left_leg[:, 0], left_leg[:, 1], left_leg[:, 2],
                    'b-', linewidth=3, marker='o', markersize=8, label='Left Leg')
        
        # Draw right leg
        right_leg = kin['right_leg']
        self.ax.plot(right_leg[:, 0], right_leg[:, 1], right_leg[:, 2],
                    'r-', linewidth=3, marker='o', markersize=8, label='Right Leg')
        
        # Label joints
        # Left leg joints
        self.ax.text(left_leg[0, 0], left_leg[0, 1], left_leg[0, 2], 'L Hip', fontsize=8)
        self.ax.text(left_leg[1, 0], left_leg[1, 1], left_leg[1, 2], 'L Knee', fontsize=8)
        self.ax.text(left_leg[2, 0], left_leg[2, 1], left_leg[2, 2], 'L Ankle', fontsize=8)
        
        # Right leg joints
        self.ax.text(right_leg[0, 0], right_leg[0, 1], right_leg[0, 2], 'R Hip', fontsize=8)
        self.ax.text(right_leg[1, 0], right_leg[1, 1], right_leg[1, 2], 'R Knee', fontsize=8)
        self.ax.text(right_leg[2, 0], right_leg[2, 1], right_leg[2, 2], 'R Ankle', fontsize=8)
        
        # Draw ground reference plane
        xx, yy = np.meshgrid(np.linspace(-0.5, 0.5, 2), np.linspace(-0.5, 0.5, 2))
        zz = np.zeros_like(xx)
        self.ax.plot_surface(xx, yy, zz, alpha=0.2, color='gray')
        
        # Display joint angle information
        info_text = "Joint Angles (radians):\n"
        info_text += f"L Hip: Yaw={self.joint_angles[0]:.3f}, Roll={self.joint_angles[2]:.3f}, Pitch={self.joint_angles[4]:.3f}\n"
        info_text += f"R Hip: Yaw={self.joint_angles[1]:.3f}, Roll={self.joint_angles[3]:.3f}, Pitch={self.joint_angles[5]:.3f}\n"
        info_text += f"L Knee: {self.joint_angles[6]:.3f} | R Knee: {self.joint_angles[7]:.3f}\n"
        info_text += f"L Ankle: Roll={self.joint_angles[8]:.3f}, Pitch={self.joint_angles[10]:.3f}\n"
        info_text += f"R Ankle: Roll={self.joint_angles[9]:.3f}, Pitch={self.joint_angles[11]:.3f}"
        
        if self.TEST_MODE:
            info_text = "MODE: TEST (Static Pose)\n" + info_text
        else:
            info_text = "MODE: TOPIC (Real-time)\n" + info_text
        
        self.ax.text2D(0.02, 0.98, info_text,
                      transform=self.ax.transAxes,
                      fontsize=8,
                      verticalalignment='top',
                      family='monospace',
                      bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
        
        self.ax.legend(loc='upper right', fontsize=9)
        
        return self.ax,
    
    def run(self):
        """Start visualization loop"""
        ani = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=50,  # 20Hz update rate
            blit=False,
            cache_frame_data=False
        )
        
        plt.show(block=True)

if __name__ == '__main__':
    try:
        visualizer = HumanoidLegVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Visualization node closed")
        pass
