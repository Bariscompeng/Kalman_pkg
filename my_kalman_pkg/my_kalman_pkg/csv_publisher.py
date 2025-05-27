import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import csv
import os
from ament_index_python.packages import get_package_share_directory

class CSVPublisher(Node):
    def __init__(self):
        super().__init__('csv_publisher')
        self.publisher_pos = self.create_publisher(Float64, '/sensor/position', 10)
        self.publisher_vel = self.create_publisher(Float64, '/sensor/velocity', 10)

        # Doğru paket ve csv yolu
        package_share_dir = get_package_share_directory('my_kalman_pkg')
        csv_path = os.path.join(package_share_dir, 'data', 'measurements.csv')

        # CSV'yi oku
        with open(csv_path, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            self.data = list(reader)

        self.index = 0
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.index < len(self.data):
            row = self.data[self.index]
            pos = float(row['z_pos_meas'])
            vel = float(row['z_vel_meas'])

            self.publisher_pos.publish(Float64(data=pos))
            self.publisher_vel.publish(Float64(data=vel))

            self.get_logger().info(f'Published pos={pos}, vel={vel}')
            self.index += 1
        else:
            self.get_logger().info('Tüm veriler yayınlandı. Yayın durduruluyor.')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = CSVPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

