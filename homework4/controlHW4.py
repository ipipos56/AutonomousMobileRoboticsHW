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

        self.scan = self.create_subscription(LaserScan, '/hagen/scan', self.scan_callback, 3)

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
        self.x_p = 0
        self.y_p = 0
        self.yaw_p = 0
        self.scan_data = []
        self.laser_range_max = 0
        self.laser_range_min = 0
        self.proj_init = False
        self.P = np.zeros((3, 4))


    def img_callback(self, m: Image):
        # Convert data to numpy array
        np_img = np.array(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)

        # Iterate over scan data and project onto image plane
        for point in self.scan_data:
            x, y, z = point

            # Project point onto image plane using camera projection matrix
            prj_pnt = self.P @ np.array([-y, 0, x, 1])
            prj_pnt = prj_pnt[:2] / prj_pnt[2]

            if z < 10:
                color = [255, 0, 0]

                # Round projected point coordinates and check if they are within image bounds
                x, y = int(round(prj_pnt[0])), int(round(prj_pnt[1]))
                if not (0 <= x < np_img.shape[1] and 0 <= y < np_img.shape[0]):
                    continue

                # Paint a square around the projected point with the computed color
                for i in range(-10, 10):
                    for j in range(-10, 10):
                        y_i = y + i
                        x_j = x + j
                        if 0 <= x_j < np_img.shape[1] and 0 <= y_i < np_img.shape[0]:
                            np_img[y_i, x_j] = color

        # Display the resulting image
        self.display(np_img)

    def camera_info_callback(self, m: CameraInfo):
        if not self.proj_init:
            self.proj_init = True
            self.P = np.reshape(m.p, (3, 4))
            print(self.P)

    def display(self, img: np.ndarray):
        cv2.imshow("camera view", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)
        pass

    def scan_callback(self, m: LaserScan):
        self.scan_data.clear()
        self.laser_range_max = m.range_max
        self.laser_range_min = m.range_min
        depths = np.array(m.ranges)

        angles = np.arange(m.angle_min, m.angle_max, m.angle_increment)
        for angle, R in zip(angles, depths):
            if R != np.inf:
                self.scan_data.append(np.array([R * np.cos(angle), R * np.sin(angle), R]))

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
        self.reference_path_follower_stanley_1step()

        return

    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.yaw_p = yaw
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

    def reference_path_follower_stanley(self, duration=50,
                                        control_points=np.array([[3, 0], [6, 5], [3, 4], [3, 1], [0, 3]],
                                                                   dtype=np.float64),
                                        k_p=0.2, k_theta=0.4):
        self.duration = duration
        self.control_points = control_points
        self.k_p = k_p
        self.k_theta = k_theta
        self.prev_ind = -1
        self.cur_ind = -1

    def reference_path_follower_stanley_1step(self):
        # Check if we are initialized in space
        if self.set_q_init is None or self.end_controller:
            return

        # Initialize current index if necessary
        if self.cur_ind == -1:
            for i in range(len(self.control_points) - 1):
                if np.linalg.norm(self.q[0:2] - self.control_points[i]) < 0.1:
                    self.cur_ind = i
                    break

        # Check if we reached the end of the path
        if self.cur_ind >= len(self.control_points) - 2:
            self.stop_vehicle()
            self.end_controller = True
            print("Finished path")
            return

        # check if the vehicle is near the current point
        d = np.linalg.norm(self.q[:2] - self.control_points[self.cur_ind])
        err = self.control_points[self.cur_ind + 1] - self.q[0:2]
        e_p = np.linalg.norm(err)

        # Check if we need to update current index
        if e_p < 0.1 and self.cur_ind < len(self.control_points) - 2:
            self.prev_ind = self.cur_ind
            print(f'Index of segment: {self.cur_ind}')
            self.cur_ind += 1
            return

        # Calculate distance to point
        dx = self.control_points[self.cur_ind][0] - self.q[0]
        dy = self.control_points[self.cur_ind][1] - self.q[1]
        d = np.sqrt(dx ** 2 + dy ** 2)

        # Calculate the angle to the point
        theta = np.arctan2(dy, dx)

        # Calculate error between current and desired angle
        error_theta = self.wrap_to_pi(theta - self.q[2])

        # P controller for velocity
        v = self.k_p * d

        # P controller for angular velocity
        w = self.k_theta * error_theta

        # Save state to history
        self.all_q.append(list(self.q))

        # Send control signal to robot
        self.send_vel(v, w)

def main(args=None):
    rclpy.init(args=args)
    control_strategy = ControlStrategy(delta_t=0.1)

    points = [[3, 0], [6, 5], [3, 4], [3, 1], [0, 3]]
    kp = 0.1
    ktheta = 0.1

    control_strategy.reference_path_follower_stanley(duration=50, control_points=np.array(points, dtype=np.float64),
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
    # info['all_rq'] = control_strategy.all_rq
    with open('info.json', 'w') as f:
        json.dump(info, f)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

# To reset ros2 service call /reset_simulation std_srvs/srv/Empty
