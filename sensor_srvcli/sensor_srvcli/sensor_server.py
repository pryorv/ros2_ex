from sensor_interfaces.srv import ReadSensor
import rclpy
from rclpy.node import Node
import socket
import numpy as np
from threading import Thread


class SensorServer(Node):
    '''
    Service server to read a number of samples at a set rate from the sensor
    '''
    def __init__(self,
        address : str,
        port : int,
        rate : int,
        instance_id : int):
        super().__init__('sensor_server')

        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect the socket to the port where the server is listening
        self.server_address = (address, port)
        print('connecting to {} port {}'.format(*self.server_address))
        self.sock.connect(self.server_address)

        self.rate = rate
        self.instance_id = instance_id
        self.srv = self.create_service(ReadSensor, 'read_sensor_' + str(instance_id), self.read_sensor_callback)

    def read_sensor_callback(self, request, response):
        self.get_logger().info('Server ' + str(self.instance_id) + ' Incoming request\nnum_samples: %d' % request.num_samples)
        message_string = str(request.num_samples)
        message = message_string.encode()
        self.sock.sendall(message)
        byte_data = self.sock.recv(10000)
        data = np.frombuffer(byte_data)
        response.samples = list(data)
        return response


def main():
    rclpy.init()
    server_rate_0 = 250
    server_rate_1 = 250

    sensor_server_0 = SensorServer('127.0.0.3', 10000, server_rate_0, 0)
    sensor_server_1 = SensorServer('127.0.0.1', 10000, server_rate_1, 1)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(sensor_server_0)
    executor.add_node(sensor_server_1)

    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate_0 = sensor_server_0.create_rate(server_rate_0)
    rate_1 = sensor_server_1.create_rate(server_rate_1)

    while rclpy.ok():
        # print('Help me body, you are my only hope')
        rate_0.sleep()


    rclpy.shutdown()
    executor_thread.join()


if __name__ == '__main__':
    main()