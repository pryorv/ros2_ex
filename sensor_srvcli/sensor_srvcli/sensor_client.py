from sensor_interfaces.srv import ReadSensor
from sensor_interfaces.msg import SensorData
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class SensorClientAsync(Node):
    """
    Simple service client to call two services to get data fr
    """
    def __init__(self):
        super().__init__('sensor_client_async')
        client_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.num_samples_0 = 5  # Number of samples to request from service server 0
        self.num_samples_1 = 5  # Number of samples to request from service server 1

        self.cli_0 = self.create_client(ReadSensor, 'read_sensor_0', callback_group=client_cb_group)
        self.cli_1 = self.create_client(ReadSensor, 'read_sensor_1', callback_group=client_cb_group)
        self.pub = self.create_publisher(SensorData, 'topic', 10)

        while not self.cli_0.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service 0 not available, waiting again...')
        while not self.cli_1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service 1 not available, waiting again...')

        self.req_0 = ReadSensor.Request()
        self.req_1 = ReadSensor.Request()
        self.msg = SensorData()

        timer_period = 1/500  # seconds
        self.timer = self.create_timer(timer_period, self.publish_timer_callback, callback_group=timer_cb_group)
        self.i = 0

    def send_request_0(self):
        self.get_logger().info(
            'Sending request for 0')
        self.req_0.num_samples = self.num_samples_0
        self.future_0 = self.cli_0.call_async(self.req_0)

    def send_request_1(self):
        self.get_logger().info(
            'Sending request for 1')
        self.req_1.num_samples = self.num_samples_1
        self.future_1 = self.cli_1.call_async(self.req_1)

    def publish_timer_callback(self):
        ## Receive response from each respective service and publish to a topic.
        # Sensor service 0
        if self.future_0.done():
            try:
                response_0 = self.future_0.result()
            except Exception as e:
                self.get_logger().info(
                    'Service call 0 failed %r' % (e,))
            else:
                self.msg.data = response_0.samples
                self.pub.publish(self.msg)
                self.get_logger().info(
                    'Read sensor 0 samples published: ' + str(self.msg.data))
                # Send another request for sensor samples when request completes
                self.send_request_0()

        # Sensor service 1
        elif self.future_1.done():
            try:
                response_1 = self.future_1.result()
            except Exception as e:
                self.get_logger().info(
                    'Service call 1 failed %r' % (e,))
            else:
                self.msg.data = response_1.samples
                self.pub.publish(self.msg)
                self.get_logger().info(
                    'Read sensor 1 samples published: ' + str(self.msg.data))
                # Send another request for sensor samples when request completes
                self.send_request_1()
        else:
            pass

def main(args=None):

    rclpy.init(args=args)

    sensor_client = SensorClientAsync()

    # Calls to service are asynchronous
    sensor_client.send_request_0()
    sensor_client.send_request_1()
    try:
        print("")
        sensor_client.get_logger().info('Beginning demo, end with CTRL-C')
        rclpy.spin(sensor_client)
    except KeyboardInterrupt:
        sensor_client.get_logger().info('KeyboardInterrupt, shutting down.\n')

    sensor_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()