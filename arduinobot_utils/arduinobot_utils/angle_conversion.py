#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from arduinobot_msgs.srv import EulerToQuaternion, QuaternionToEuler 
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class AngleConverter(Node):
    def __init__(self):
        super().__init__('angle_conversion_service_server')

        self.euler_to_quaternion_ = self.create_service(
            EulerToQuaternion,
            'euler_to_quaternion',
            self.euler_to_quaternion_callback
        )

        self.quaternion_to_euler_ = self.create_service(
            QuaternionToEuler,
            'quaternion_to_euler',
            self.quaternion_to_euler_callback
        )

        self.get_logger().info('Angle conversion service has been started')


    def euler_to_quaternion_callback(self, request, response):
        self.get_logger().info('Requested to convert euler angles roll: {}, pitch: {}, yaw: {} to quaternion'.format(request.roll, request.pitch, request.yaw))
        (response.x, response.y, response.z, response.w) = quaternion_from_euler(request.roll, request.pitch, request.yaw)
        self.get_logger().info('Converted quaternion x: {}, y: {}, z: {}, w: {}'.format(response.x, response.y, response.z, response.w))
        return response

    def quaternion_to_euler_callback(self, request, response):
        self.get_logger().info('Requested to convert quaternion x: {}, y: {}, z: {}, w: {} to euler angles'.format(request.x, request.y, request.z, request.w))
        (response.roll, response.pitch, response.yaw) = euler_from_quaternion([request.x, request.y, request.z, request.w])
        self.get_logger().info('Converted euler angles roll: {}, pitch: {}, yaw: {}'.format(response.roll, response.pitch, response.yaw))
        return response

def main():
    rclpy.init()
    angle_converter = AngleConverter()
    rclpy.spin(angle_converter)
    angle_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()