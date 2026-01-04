#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


class JointToTwoServos(Node):
    def __init__(self):
        super().__init__('joint_to_two_servos')

        # list with my joints
        
        self.target_joints = ['Revolute_1', 'Revolute_6', 'Revolute_2', 'Revolute_4', 'Revolute_5']

        # subscription on /joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )

        # publication of angles
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/servo_angles',   # new_topic
            10
        )

    
    # initializes every time when new message in /joint_states
    def listener_callback(self, msg: JointState):
        
        angles_deg = []

        for joint_name in self.target_joints:
            if joint_name in msg.name:
                #msg.name список в /joint_states
                idx = msg.name.index(joint_name)
                rad = msg.position[idx]
                deg = math.degrees(rad)
                angles_deg.append(deg)
            else:
                self.get_logger().warn(f'{joint_name} not found')
                return
            
        angles_deg[0] = (angles_deg[0]-90)*(-1)
        angles_deg[1] = (angles_deg[1]-90)*(-1)
        angles_deg[2] = angles_deg[2]+90
        angles_deg[3] = angles_deg[3]+90

        # publishing
        out_msg = Float32MultiArray()
        out_msg.data = angles_deg
        self.publisher.publish(out_msg)

        self.get_logger().info(
            f'{self.target_joints[0]} = {angles_deg[0]:.2f}°,'
            f'{self.target_joints[1]} = {angles_deg[1]:.2f}°,'
            f'{self.target_joints[2]} = {angles_deg[2]:.2f}°,'
            f'{self.target_joints[3]} = {angles_deg[3]:.2f}°,'
            f'{self.target_joints[4]} = {angles_deg[4]:.2f}°'

        )


def main(args=None):
    rclpy.init(args=args)
    node = JointToTwoServos()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()