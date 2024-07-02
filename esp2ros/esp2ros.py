import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class ESP32Listener(Node):

    def __init__(self):
        super().__init__('esp32_listener')
        self.publisher_ = self.create_publisher(String, 'esp32/input', 10)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('', 1883))  # IP of ROS2 PC
        self.server_socket.listen(5)
        self.get_logger().info("Socket server listening on port 1883")

        # Dictionary to store the state of each pin
        self.pin_states = {}

        # Create a service to get the current state of all pins
        self.srv = self.create_service(
            Trigger, 'get_pin_states', self.get_pin_states_callback)

    def start_listening(self):
        while rclpy.ok():
            client_socket, addr = self.server_socket.accept()
            self.get_logger().info(f"Connection from {addr}")
            while rclpy.ok():
                data = client_socket.recv(1024).decode()
                if not data:
                    break
                self.get_logger().info(f"Received message: {data}")
                self.update_pin_states(data)
                ros_msg = String()
                ros_msg.data = data
                self.publisher_.publish(ros_msg)
            client_socket.close()

    def update_pin_states(self, message):
        # Parse the message to update pin states
        try:
            parts = message.split()
            pin = int(parts[1])
            state = int(parts[4])
            self.pin_states[pin] = state
        except (IndexError, ValueError) as e:
            self.get_logger().error(
                f"Failed to parse message: {message} with error: {e}")

    def get_pin_states_callback(self, request, response):
        # Create a response message with the current pin states
        pin_states_str = '\n'.join(
            [f"Pin {pin}: {state}" for pin, state in self.pin_states.items()])
        if pin_states_str:
            response.success = True
            response.message = pin_states_str
        else:
            response.success = False
            response.message = "No pin states available."
        return response


def main(args=None):
    rclpy.init(args=args)
    esp32_listener = ESP32Listener()
    try:
        esp32_listener.start_listening()
    except KeyboardInterrupt:
        esp32_listener.server_socket.close()
        pass
    esp32_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
