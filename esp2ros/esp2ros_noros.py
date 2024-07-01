import socket


class ESP32Listener:

    def __init__(self, host, port):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((host, port))
        self.server_socket.listen(5)
        print(f"Socket server listening on {host}:{port}")

    def start_listening(self):
        try:
            while True:
                client_socket, addr = self.server_socket.accept()
                print(f"Connection from {addr}")
                while True:
                    data = client_socket.recv(1024).decode()
                    if not data:
                        break
                    print(f"Received message: {data}")
                client_socket.close()
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            self.server_socket.close()


def main():
    host = '192.168.1.123'  # Replace with your PC's IP address
    port = 1883
    esp32_listener = ESP32Listener(host, port)
    esp32_listener.start_listening()


if __name__ == '__main__':
    main()
