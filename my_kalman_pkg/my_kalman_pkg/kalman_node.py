import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np
from my_kalman_pkg.filter_params import A, H, Q, R, P0

import csv
import time
from pathlib import Path

class KalmanNode(Node):
    def __init__(self):
        super().__init__('kalman_node')

        self.state = np.zeros((2, 1))
        self.P = P0.copy()

        self.pos_sub = self.create_subscription(Float64, '/sensor/position', self.pos_callback, 10)
        self.vel_sub = self.create_subscription(Float64, '/sensor/velocity', self.vel_callback, 10)

        self.pos_pub = self.create_publisher(Float64, '/state/position_est', 10)
        self.vel_pub = self.create_publisher(Float64, '/state/velocity_est', 10)

        self.z_pos_buffer = []
        self.z_vel_buffer = []

        self.max_steps = 10
        self.current_step = 0

        self.csv_path = str(Path.home() / "estimates.csv")
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'position_est', 'velocity_est'])

    def pos_callback(self, msg):
        self.z_pos_buffer.append(msg.data)
        self.try_process()

    def vel_callback(self, msg):
        self.z_vel_buffer.append(msg.data)
        self.try_process()

    def try_process(self):
        if self.current_step >= self.max_steps:
            return

        if len(self.z_pos_buffer) == 0 or len(self.z_vel_buffer) == 0:
            return

        z_pos = self.z_pos_buffer.pop(0)
        z_vel = self.z_vel_buffer.pop(0)

        self.predict()

        z = np.array([[z_pos], [z_vel]])
        self.update(z)

        pos_est = float(self.state[0, 0])
        vel_est = float(self.state[1, 0])

        self.pos_pub.publish(Float64(data=pos_est))
        self.vel_pub.publish(Float64(data=vel_est))

        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                time.time(),
                pos_est,
                vel_est
            ])

        self.get_logger().info(f"Tahmin #{self.current_step + 1}: pos={pos_est:.2f}, vel={vel_est:.2f}")
        self.current_step += 1

    def predict(self):
        self.state = A @ self.state
        self.P = A @ self.P @ A.T + Q

    def update(self, z):
        y = z - H @ self.state
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(2) - K @ H) @ self.P

def main(args=None):
    rclpy.init(args=args)
    node = KalmanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

