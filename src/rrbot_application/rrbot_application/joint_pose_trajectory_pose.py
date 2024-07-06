import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.qos import qos_profile_system_default


class MyNode(Node):
    def __init__(self):
        node_name="minimal"
        super().__init__(node_name)
        self.pub = self.create_publisher(JointTrajectory,
                                         "/set_joint_trajectory",
                                         qos_profile=qos_profile_system_default
                                         )
        
        self.send_position()
        self.get_logger().info("Hello ROS2")

    def send_position(self):
        msg = JointTrajectory()
        header = Header()
        header.frame_id = "world"
        
        msg.header = header
        msg.joint_names = ["joint1", "joint2"]
        
        points = JointTrajectoryPoint()
        points.positions = [1.0, 1.0]
        points.velocities = []
        points.accelerations = []
        points.effort = []
        # points.time_from_start = Duration(sec=10)

        msg.points = [points]
        self.pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()