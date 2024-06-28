import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ESP32Listener(Node):

    def __init__(self):
        super().__init__('esp32_listener')
        self.publisher_ = self.create_publisher(String, 'esp32/input', 10)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('192.168.1.100', 1883)) # IP of ROS2 PC
        self.server_socket.listen(5)
        self.get_logger().info("Socket server listening on port 1883")

    def start_listening(self):
        while rclpy.ok():
            client_socket, addr = self.server_socket.accept()
            self.get_logger().info(f"Connection from {addr}")
            while rclpy.ok():
                data = client_socket.recv(1024).decode()
                if not data:
                    break
                self.get_logger().info(f"Received message: {data}")
                ros_msg = String()
                ros_msg.data = data
                self.publisher_.publish(ros_msg)
            client_socket.close()

def main(args=None):
    rclpy.init(args=args)
    esp32_listener = ESP32Listener()
    try:
        esp32_listener.start_listening()
    except KeyboardInterrupt:
        pass
    esp32_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
