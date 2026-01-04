import ikpy.chain
import numpy as np
import time
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = ['Revolute_1', 'Revolute_6', 'Revolute_2', 'Revolute_4']
TOPIC_NAME = '/joint_command'
PUB_RATE_HZ = 30  


my_chain = ikpy.chain.Chain.from_urdf_file(
    "/home/dmitriyb51/Downloads/Untitled_description/urdf/Untitled.urdf",
    active_links_mask=[False, True, True, True, True, False]
)

POSITION_TOLERANCE = 0.04

ORIENTATIONS = [
    [0, 0, -1],
    [1, 0, 0],
    [-1, 0, 0],
    [0, 1, 0],
    [0, -1, 0],
]


class IsaacJointSender(Node):
    def __init__(self):
        super().__init__('ikpy_to_isaac_sender')
        self.pub = self.create_publisher(JointState, TOPIC_NAME, 10)

    def send_angles(self, angles_4):
        
        msg = JointState()
        msg.name = JOINT_NAMES
        msg.position = list(map(float, angles_4))
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = IsaacJointSender()

    dt = 1.0 / PUB_RATE_HZ

    while True:
        A = float(input("X coordinate: "))
        B = float(input("Y coordinate: "))
        C = float(input("Z coordinate: "))

        target_position = [A, B, C]
        success = False

        for target_orientation in ORIENTATIONS:
            print(f"\nTrying orientation: {target_orientation}")

            ik = my_chain.inverse_kinematics(
                target_position,
                target_orientation,
                orientation_mode="Y"
            )

            computed_position = my_chain.forward_kinematics(ik)
            ee_pos = computed_position[:3, 3]

            print("Computed position:", [f"{x:.3f}" for x in ee_pos])

            if (
                abs(ee_pos[0] - A) <= POSITION_TOLERANCE and
                abs(ee_pos[1] - B) <= POSITION_TOLERANCE and
                abs(ee_pos[2] - C) <= POSITION_TOLERANCE
            ):
                print("Position reached successfully!")


                
                angles_4 = ik[1:5]
                

                print("Joint angles (deg):", [round(math.degrees(r), 2) for r in angles_4])


                for _ in range(int(PUB_RATE_HZ * 0.5)):  
                    node.send_angles(angles_4)
                    rclpy.spin_once(node, timeout_sec=0.0)
                    time.sleep(dt)

                success = True
                break

        if not success:
            print("position is not reachable")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
