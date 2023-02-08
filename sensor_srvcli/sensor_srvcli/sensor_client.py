from sensor_interfaces.srv import ReadSensor
from sensor_interfaces.msg import SensorData
import sys
import rclpy
from rclpy.node import Node
from threading import Thread
import argparse

class SensorClientAsync(Node):
    """
    Simple service client to call two services to get data fr
    """
    def __init__(self):
        super().__init__('sensor_client_async')
        self.cli_0 = self.create_client(ReadSensor, 'read_sensor_0')
        self.cli_1 = self.create_client(ReadSensor, 'read_sensor_1')
        self.pub = self.create_publisher(SensorData, 'topic', 10)

        while not self.cli_0.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service 0 not available, waiting again...')
        while not self.cli_1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service 1 not available, waiting again...')

        self.req_0 = ReadSensor.Request()
        self.req_1 = ReadSensor.Request()
        self.msg = SensorData()

    def send_request_0(self, num_samples_0):
        self.req_0.num_samples = num_samples_0
        self.future_0 = self.cli_0.call_async(self.req_0)

    def send_request_1(self, num_samples_1):
        self.req_1.num_samples = num_samples_1
        self.future_1 = self.cli_1.call_async(self.req_1)

    def publish_data(self, data):
        self.msg.data = data
        self.pub.publish(self.msg)

def main(args=None):
    num_samples0 = 5 #Number of samples to request from service server 0
    num_samples1 = 5 #Number of samples to request from service server 1
    rclpy.init(args=args)

    sensor_client = SensorClientAsync()
    rate = sensor_client.create_rate(500)

    # Calls to service are asynchronous
    sensor_client.send_request_0(num_samples0)
    sensor_client.send_request_1(num_samples1)

    while rclpy.ok():
        rclpy.spin_once(sensor_client)

        ## Receive response from each respective service and publish to a topic.
        # Sensor service 0
        if sensor_client.future_0.done():
            try:
                response_0 = sensor_client.future_0.result()
            except Exception as e:
                sensor_client.get_logger().info(
                    'Service call 0 failed %r' % (e,))
            else:
                sensor_client.publish_data(response_0.samples)
                sensor_client.get_logger().info(
                    'Read sensor 0 samples published: ' + str(sensor_client.msg.data))
                # Send another request for sensor samples when request completes
                sensor_client.send_request_0(num_samples0)

        # Sensor service 1
        if sensor_client.future_1.done():
            try:
                response_1 = sensor_client.future_1.result()
            except Exception as e:
                sensor_client.get_logger().info(
                    'Service call 1 failed %r' % (e,))
            else:
                sensor_client.publish_data(response_1.samples)
                sensor_client.get_logger().info(
                    'Read sensor 1 samples published: ' + str(sensor_client.msg.data))
                # Send another request for sensor samples when request completes
                sensor_client.send_request_1(num_samples1)
    sensor_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()