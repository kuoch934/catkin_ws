#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import pygame

def main():
    pygame.init()
    pygame.joystick.init()

    js = pygame.joystick.Joystick(0)
    js.init()
    print("🎮 Connected:", js.get_name())

    pub = rospy.Publisher('/cmd_vel', Float32MultiArray, queue_size=10)
    cmd = Float32MultiArray()
    rospy.init_node('ps5_to_cmdvel')
    rate = rospy.Rate(50)  # 50 Hz

    while not rospy.is_shutdown():
        pygame.event.pump()

        # PS5 軸對應
        lx = js.get_axis(0)   # 左搖桿 X（左/右）
        ly = -js.get_axis(1)  # 左搖桿 Y（前/後）
        rx = js.get_axis(3)   # 右搖桿 X（旋轉）

        cmd.data = [ly * 0.5, lx * 0.5, rx * 1.0]
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    main()
