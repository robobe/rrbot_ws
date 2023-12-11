import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


AMPLITUDE = 30
FREQ = 1 / 5
TOPIC = "/joint_states"

class MyNode(Node):
    def __init__(self):
        node_name = "sim_joint_state"
        super().__init__(node_name)
        self.create_timer(FREQ, self.timer_handler)
        self.joint_pub = self.create_publisher(
            JointState, TOPIC, qos_profile=qos_profile_sensor_data
        )

    def timer_handler(self):
        deg = AMPLITUDE * math.sin(time.time() * FREQ)
        rad = math.radians(deg)
        msg = JointState()
        msg.header = Header()
        msg.header.frame_id = ""
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint1", "joint2"]
        msg.position = [rad, 0.0]
        msg.velocity = []
        msg.effort = []
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
