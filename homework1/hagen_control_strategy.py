# GitHubLink
# https://github.com/ipipos56/AutonomousMobileRoboticsHW

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import numpy as np
import os

import matplotlib.pyplot as plt


class ControlStrategy(Node):
    def __init__(self, delta_t, duration=3):
        super().__init__('control_strategy')
        self.publisher_ = self.create_publisher(Twist, '/hagen/cmd_vel', 30)
        self.vel_sub = self.create_subscription(Twist, '/hagen/cmd_vel', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/hagen/odom", self.set_pose, 20)
        self.vel_sub
        self.odom_sub
        self.i = 0
        self.set_q_init = None
        self.q = None
        self.duration = duration
        self.r = 0.04
        self.L = 0.08
        self.D = 0.07  # Distance between the front whell and rear axle
        self.Ts = delta_t  # Sampling time
        self.t = np.arange(0, duration, self.Ts)  # Simulation time
        self.end_controller = False
        self.timer = self.create_timer(self.Ts, self.timer_callback)
        self.v = 0
        self.w = 0
        self.q_info = []
        self.sim_q_info = []

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def listener_callback(self, msg):
        pass

    def stop_vehicle(self, ):
        self.send_vel(0.0, 0.0)

    def diff_drive_init(self, v=3, w=1, duration=4):
        self.q_info = []
        self.sim_q_info = []
        self.duration = duration
        self.v = v
        self.w = w
        self.time_utilized = 0.0

    def timer_callback(self):
        self.time_utilized += self.Ts

        if self.time_utilized < self.duration:
            print("send vel" + str(self.v) + " " + str(self.w))
            self.send_vel(self.v, self.w)
            return

        self.sim_q_info.append(self.q)
        self.stop_vehicle()
        self.end_controller = True
        print("i have finished callback")

        time.sleep(1)
        os.system('ros2 service call /reset_simulation std_srvs/srv/Empty "{}"')
        time.sleep(1)

        # self.const_param_one_step()
        # self.perform_action_diff_drive_one_step()
        # self.inter_direction_diff_drive()
        # self.inter_point_diff_drive()
        return

    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.q_info.append(np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]))
        if (self.set_q_init is None):
            self.set_q_init = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
            self.q = self.set_q_init
            self.sim_q_info.append(self.set_q_init)

    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.publisher_.publish(msg)


def simulate_all(args, ang_lin_vel):
    rclpy.init(args=args)
    control_strategy = ControlStrategy(delta_t=0.03)
    # control_strategy.diff_drive_init()
    # control_strategy.inter_point_diff_drive_init()
    # control_strategy.inter_direction_diff_drive_init()
    control_strategy.diff_drive_init(v=ang_lin_vel[0], w=ang_lin_vel[1])
    while control_strategy.end_controller is False and rclpy.ok():
        try:
            rclpy.spin_once(control_strategy)
        except KeyboardInterrupt:
            break

    # info = control_strategy.q_info
    # sim_info = control_strategy.sims_q_info
    info = control_strategy.q_info
    sim_info = control_strategy.sim_q_info

    # control_strategy.stop_vehicle()
    # control_strategy.stop_vehicle()
    # control_strategy.stop_vehicle()
    # control_strategy.stop_vehicle()


    control_strategy.destroy_node()
    rclpy.shutdown()

    return info, sim_info


L = 0.08


def vlvr_to_vw(vl, vr):
    v = (vr + vl) / 2
    w = (vr - vl) / L
    return v, w


def main(args=None):
    ang_lin_vel_args = [[0.5, 0.0],
                        [1.0, 2.0],
                        [0.0, 2.0]]

    left_right_vel = [20.0, 18.0]

    info = []
    sims = []

    for ang_lin_vel in ang_lin_vel_args:
        q_info, sims_info = simulate_all(args, ang_lin_vel)
        info.append(q_info)
        sims.append(sims_info)

    q_info, sims_info = simulate_all(args, vlvr_to_vw(*left_right_vel))
    info.append(q_info)
    sims.append(sims_info)

    print(info)
    print("______________________")
    print(sims)


    with open('info.txt', 'w') as outfile:
        for t in info:
            for i in t:
                outfile.write(f'{i[0]} {i[1]} ')
            outfile.write('\n')

    with open('sims.txt', 'w') as outfile:
        for t in sims:
            for i in t:
                outfile.write(f'{i[0]} {i[1]} \n')

if __name__ == '__main__':
    main()

# To reset ros2 service call /reset_simulation std_srvs/srv/Empty
