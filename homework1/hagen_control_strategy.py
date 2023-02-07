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

    def wrap_to_pi(self, x):
        x = np.array([x])
        xwrap = np.remainder(x, 2 * np.pi)
        mask = np.abs(xwrap) > np.pi
        xwrap[mask] -= 2 * np.pi * np.sign(xwrap[mask])
        return xwrap[0]

    def diff_drive_init(self, duration=10):
        self.duration = duration
        self.wL = 12  # Left wheel velocity
        self.wR = 12  # Right wheel velocity
        self.time_utilized = 0.0

    def inter_point_diff_drive_init(self, duration=10, r_distance=1.3
                                    , refPose=np.array([3, 6, 0]), k_p=0.5, k_w=0.7, dmin=0.7):

        self.duration = duration
        self.r_distance = r_distance
        self.refPose = refPose
        self.k_p = k_p
        self.k_w = k_w
        self.dmin = dmin
        self.time_utilized = 0.0
        self.xT = self.refPose[0] - self.r_distance * np.cos(self.refPose[2])
        self.yT = self.refPose[1] - self.r_distance * np.sin(self.refPose[2])
        self.state = 0

    def inter_direction_diff_drive_init(self, duration=10, r_distance=1.3
                                        , refPose=np.array([3, 6, 0]), k_p=0.5, k_w=0.7, dmin=0.7):
        self.duration = duration
        self.r_distance = r_distance
        self.refPose = refPose
        self.k_p = k_p
        self.k_w = k_w
        self.dmin = dmin
        self.time_utilized = 0.0

    def inter_direction_diff_drive(self, ):
        if (self.q is not None):
            if (self.duration < self.time_utilized):
                print("End of simulation")
                self.send_vel(0.0, 0.0)
                self.end_controller = True

            self.D = np.sqrt((self.q[0] - self.refPose[0]) ** 2
                             + (self.q[1] - self.refPose[1]) ** 2)

            if (self.D < self.dmin):
                print("Reach to the goal pose")
                self.send_vel(0.0, 0.0)
                self.end_controller = True

            beta = np.arctan(self.r_distance / self.D)
            phiR = np.arctan2(self.refPose[1] - self.q[1], self.refPose[0] - self.q[0])
            alpha = self.wrap_to_pi(phiR - self.refPose[2])

            if (alpha < 0):
                beta = -beta
            ##Controller
            if (np.abs(alpha) < np.abs(beta)):
                ePhi = self.wrap_to_pi(phiR - self.q[2] + alpha)
            else:
                ePhi = self.wrap_to_pi(phiR - self.q[2] + beta)

            v = self.k_p * self.D
            w = self.k_w * ePhi
            print("Distance to the goal: ", self.D)
            dq = np.array([v * np.cos(self.q[2] + self.Ts * w / 2)
                              , v * np.sin(self.q[2] + self.Ts * w / 2), w])
            self.q = self.q + self.Ts * dq  # Integration
            self.q[2] = self.wrap_to_pi(self.q[2])  # Map orientation angle to [-pi, pi]
            self.send_vel(v, w)
            self.time_utilized = self.time_utilized + self.Ts

    def inter_point_diff_drive(self, ):
        if (self.q is not None):
            if (self.duration < self.time_utilized):
                self.stop_vehicle()
                print("End of simulation")
                self.end_controller = True

            self.D = np.sqrt((self.q[0] - self.refPose[0]) ** 2 + (self.q[1] - self.refPose[1]) ** 2)

            if (self.D < self.dmin):
                self.stop_vehicle()
                print("Reach to the goal pose")
                self.end_controller = True

            if self.state == 0:
                d = np.sqrt((self.yT - self.q[1]) ** 2 + (self.xT - self.q[0]) ** 2)
                if (d < self.dmin):
                    self.state = 1
                self.phiT = np.arctan2(self.yT - self.q[1], self.xT - self.q[0])
                self.ePhi = self.phiT - self.q[2]
            else:
                self.ePhi = self.refPose[2] - self.q[2]

            v = self.k_p * self.D
            w = self.k_w * self.ePhi
            print("Distance to the goal: ", self.D)
            dq = np.array([v * np.cos(self.q[2] + self.Ts * w / 2), v * np.sin(self.q[2] + self.Ts * w / 2), w])
            self.q = self.q + self.Ts * dq  # Integration
            self.q[2] = self.wrap_to_pi(self.q[2])  # Map orientation angle to [-pi, pi]
            self.send_vel(v, w)
            self.time_utilized = self.time_utilized + self.Ts

    def perform_action_diff_drive_one_step(self):
        v = self.r / 2 * (self.wR + self.wL)  # Robot velocity
        w = self.r / self.L * (self.wR - self.wL)  # Robot angular velocity
        dq = np.array([v * np.cos(self.q[2] + self.Ts * w / 2), v * np.sin(self.q[2] + self.Ts * w / 2), w])
        self.q = self.q + self.Ts * dq  # Integration
        self.q[2] = self.wrap_to_pi(self.q[2])  # Map orientation angle to [-pi, pi]
        self.send_vel(v, w)
        self.time_utilized = self.time_utilized + self.Ts

    def const_param_move(self, lin_ang_vel):



    def const_param_one_step(self, lin_ang_vel):



    def timer_callback(self, ):
        self.const_param_one_step()
        # self.perform_action_diff_drive_one_step()
        # self.inter_direction_diff_drive()
        # self.inter_point_diff_drive()
        return

    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        if (self.set_q_init is None):
            self.set_q_init = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
            self.q = self.set_q_init

    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher_.publish(msg)


def simulate_all(args, ang_lin_vel):
    rclpy.init(args=args)
    control_strategy = ControlStrategy(delta_t=0.03)
    # control_strategy.diff_drive_init()
    # control_strategy.inter_point_diff_drive_init()
    # control_strategy.inter_direction_diff_drive_init()
    control_strategy.const_param_move(ang_lin_vel)
    while control_strategy.end_controller is False and rclpy.ok():
        try:
            rclpy.spin_once(control_strategy)
        except KeyboardInterrupt:
            break

    info = control_strategy.q_info
    sim_info = control_strategy.sims_q_info

    control_strategy.stop_vehicle()
    control_strategy.stop_vehicle()
    control_strategy.stop_vehicle()
    control_strategy.stop_vehicle()

    time.sleep(1)
    os.system('ros2 service call /reset_simulation std_srvs/srv/Empty "{}"')
    time.sleep(1)

    control_strategy.destroy_node()
    rclpy.shutdown()

    return info, sim_info


def main(args=None):
    ang_lin_vel_args = [[0.5, 0.0],
                        [1.0, 2.0],
                        [0.0, 2.0]]

    left_right_vel = [20.0, 18.0]

    info = []
    sims = []
    analyt = []

    for ang_lin_vel in ang_lin_vel_args:
        q_info, sims_info = simulate_all(args=args, delta_t=delta_t, r=r, L=L, duration=duration, velocity=velocity)
        dumps.append(q_dump)
        ref.append(ref_dump)
        analytical.append(analytical_solution(velocity, delta_t, duration, r, L))


if __name__ == '__main__':
    main()

# To reset ros2 service call /reset_simulation std_srvs/srv/Empty
