import rosbag2_py
import rclpy
from std_msgs.msg import Float64
from rclpy.serialization import deserialize_message
import matplotlib.pyplot as plt

bag_path = '/home/baris/ros2_ws/rosbag2_2025_05_27-00_40_20'

storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('', '')

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

topic_types = {
    '/sensor/position': Float64,
    '/sensor/velocity': Float64,
    '/state/position_est': Float64,
    '/state/velocity_est': Float64
}


sensor_pos = []
sensor_vel = []
state_pos = []
state_vel = []
timestamps = []

while reader.has_next():
    (topic, data, t) = reader.read_next()
    msg_type = topic_types.get(topic)
    if not msg_type:
        continue
    msg = deserialize_message(data, msg_type)
    
    timestamps.append(t / 1e9)  
    
    if topic == '/sensor/position':
        sensor_pos.append(msg.data)
    elif topic == '/sensor/velocity':
        sensor_vel.append(msg.data)
    elif topic == '/state/position_est':
        state_pos.append(msg.data)
    elif topic == '/state/velocity_est':
        state_vel.append(msg.data)


plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.plot(sensor_pos, 'r-', label='Sensor Position')
plt.plot(state_pos, 'b--', label='Estimated Position')
plt.legend()
plt.title('Position')

plt.subplot(2, 1, 2)
plt.plot(sensor_vel, 'r-', label='Sensor Velocity')
plt.plot(state_vel, 'b--', label='Estimated Velocity')
plt.legend()
plt.title('Velocity')

plt.tight_layout()
plt.show()

