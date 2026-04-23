#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import copy

class ImuCovarianceFix(Node):
    def __init__(self):
        super().__init__('imu_covariance_fix')
        self.subscriber = self.create_subscription(Imu, '/imu/data', self.callback, 10)
        self.publisher = self.create_publisher(Imu, '/imu/data_corrected', 10)

    def callback(self, msg: Imu):
        # Create a deep copy of the incoming message
        corrected_msg = copy.deepcopy(msg)
        
        # Replace zero covariance with nonzero values
        if all(v == 0.0 for v in corrected_msg.orientation_covariance):
            corrected_msg.orientation_covariance = [0.1, 0.0, 0.0,
                                                     0.0, 0.1, 0.0,
                                                     0.0, 0.0, 0.1]
        if all(v == 0.0 for v in corrected_msg.angular_velocity_covariance):
            corrected_msg.angular_velocity_covariance = [0.1, 0.0, 0.0,
                                                          0.0, 0.1, 0.0,
                                                          0.0, 0.0, 0.1]
        if all(v == 0.0 for v in corrected_msg.linear_acceleration_covariance):
            corrected_msg.linear_acceleration_covariance = [0.1, 0.0, 0.0,
                                                             0.0, 0.1, 0.0,
                                                             0.0, 0.0, 0.1]
        self.publisher.publish(corrected_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
