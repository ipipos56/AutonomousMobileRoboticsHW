import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
import std_msgs.msg as std_msg
import rclpy.qos as qos


class ControlStrategy(Node):
    def __init__(self, delta_t, ):
        super().__init__('control_strategy')
        self.publisher_ = self.create_publisher(Twist, '/hagen/cmd_vel', 30)
        self.vel_sub = self.create_subscription(Twist, '/hagen/cmd_vel', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/hagen/odom", self.set_pose, 20)

        self.img_sub = self.create_subscription(Image
                                                , '/depth_camera/image_raw', self.img_callback, 3)

        self.img_info_sub = self.create_subscription(CameraInfo
                                                     , '/depth_camera/camera_info', self.camera_info_callback, 3)

        self.scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 3)

        self.all_q = []
        self.all_rq = []
        self.i = 0
        self.set_q_init = None
        self.q = None
        self.r = 0.3  # Wheel radius
        self.L = 1.25  # Axle length
        self.D = 0.07  # Distance between the front wheel and rear axle
        self.Ts = delta_t  # Sampling time
        self.t = np.arange(0, 10, self.Ts)  # Simulation time
        self.end_controller = False
        self.timer = self.create_timer(self.Ts, self.timer_callback)

    def img_callback(self, m: Image):
        np_img = np.reshape(m.data, (m.height, m.width, 3)).astype(np.uint8)
        self.display(np_img)

    def camera_info_callback(self, m: CameraInfo):
        # print(m)
        pass

    def display(self, img: np.ndarray):
        # cv2.imshow("camera view", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        # cv2.waitKey(1)
        pass

    def scan_callback(self, m: LaserScan):
        scan_data = np.array(m.ranges)

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

    def timer_callback(self, ):
        self.reference_path_follower_diff_drive_1step()

        return

    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        if (self.set_q_init is None):
            self.set_q_init = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
            self.all_rq.append(self.set_q_init)
            self.all_q.append(self.set_q_init)
            self.q = np.array(self.set_q_init)
        else:
            self.all_rq.append([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])

    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher_.publish(msg)

    def reference_path_follower_diff_drive(self, duration=50,
                                           control_points=np.array([[3, 0], [6, 5], [3, 4], [3, 1], [0, 3]],
                                                                   dtype=np.float64),
                                           k_p=0.2, k_theta=0.4):
        self.duration = duration
        self.control_points = control_points
        self.k_p = k_p
        self.k_theta = k_theta
        self.prev_ind = -1
        self.cur_ind = -1

    def reference_path_follower_diff_drive_1step(self):
        # Check if we are initialized in space
        if self.set_q_init is None or self.end_controller:
            return

        # Initialize current index if necessary
        if self.cur_ind == -1:
            for i in range(len(self.control_points) - 1):
                a1 = np.dot(self.q[:2] - self.control_points[i],
                            self.control_points[self.cur_ind + 1] - self.control_points[i])
                a2 = np.dot(self.control_points[i + 1] - self.control_points[i],
                            self.control_points[i + 1] - self.control_points[i])
                if a1 / a2 < 1:
                    self.cur_ind = i
                    break

        # Check if we have reached the end of the path
        if self.cur_ind == len(self.control_points) - 2:
            a = np.sqrt((self.control_points[self.cur_ind + 1][1] - self.q[1]) ** 2 + (
                    self.control_points[self.cur_ind + 1][0] - self.q[0]) ** 2)
            if a < 1:
                for i in range(100):
                    self.stop_vehicle()
                print("Finished path")
                self.end_controller = True
                return

        # Check if we need to switch to the next line segment
        a1 = np.dot(self.q[:2] - self.control_points[self.cur_ind],
                    self.control_points[self.cur_ind + 1] - self.control_points[self.cur_ind])
        a2 = np.dot(self.control_points[self.cur_ind + 1] - self.control_points[self.cur_ind],
                    self.control_points[self.cur_ind + 1] - self.control_points[self.cur_ind])
        if abs(a1) / abs(a2) > 1:
            self.cur_ind += 1

        # Compute desired heading angle and update robot state
        segment = self.control_points[self.cur_ind + 1] - self.control_points[self.cur_ind]
        a = np.array([segment[1], -segment[0]])
        a2 = np.dot(a, self.q[:2] - self.control_points[self.cur_ind]) / np.dot(a, a)
        p_seg = np.arctan2(segment[1], segment[0])
        p_rot = np.arctan(0.5 * a2)

        p_ref = p_seg + p_rot
        p_err = self.wrap_to_pi(p_ref - self.q[2])
        v = self.k_p * np.cos(p_err)
        w = self.k_theta * p_err
        a = np.array([v * np.cos(self.q[2] + self.Ts * w / 2), v * np.sin(self.q[2] + self.Ts * w / 2), w])
        self.q = self.q + self.Ts * a
        self.q[2] = self.wrap_to_pi(self.q[2])
        self.all_q.append(list(self.q))

        # Print debug information
        if self.cur_ind != self.prev_ind:
            print(f'Index of segment: {self.cur_ind}, segment: {segment}, p_ref: {p_ref}')
            self.prev_ind = self.cur_ind

        # Send velocity commands to robot
        self.send_vel(v, w)


def main(args=None):
    rclpy.init(args=args)
    control_strategy = ControlStrategy(delta_t=0.03)

    points = [[3, 0], [6, 5], [3, 4], [3, 1], [0, 3]]
    kp = 0.4
    ktheta = 3

    control_strategy.reference_path_follower_diff_drive(duration=50, control_points=np.array(points, dtype=np.float64),
                                                        k_p=kp, k_theta=ktheta)

    while rclpy.ok():
        try:
            rclpy.spin_once(control_strategy)
        except KeyboardInterrupt:
            break
    control_strategy.destroy_node()

    import json
    info = dict()
    info['all_q'] = control_strategy.all_q
    info['all_rq'] = control_strategy.all_rq
    with open('info.json', 'w') as f:
        json.dump(info, f)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

# To reset ros2 service call /reset_simulation std_srvs/srv/Empty
