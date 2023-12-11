import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rrbot_application.utils import euler_from_quaternion, rad2deg, deg2rad
from rrbot_application.pid_controller import PID
from std_msgs.msg import Float64MultiArray
import math
import time

TOPIC = "/imu_sensor_broadcaster/imu"
TOPIC_EFFORT = "/effort_controller/commands"
TOPIC_POSITION = "/position_controller/commands"
POS_FREQ = 1
POS_AMP = 30

class MyNode(Node):
    def __init__(self):
        node_name = "minimal_pub"
        super().__init__(node_name)
        self.create_timer(0.1, self.timer_handler)
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            TOPIC_EFFORT,
            qos_profile_sensor_data
        )
        self.pos_pub = self.create_publisher(
            Float64MultiArray,
            TOPIC_POSITION,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, TOPIC, self.imu_handler, qos_profile=qos_profile_system_default
        )
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.01)
        self.get_logger().info("run simple pub")
        self.pid = PID(Kp=0.0, Ki=0.0, Kd=0.0, target=0)
        self.pid.set_limits(min_int=-10, max_int=10)
        self.add_on_set_parameters_callback(self.parameter_callback)

    def timer_handler(self):
        pos_msg = Float64MultiArray()
        value = POS_AMP * math.sin(POS_FREQ*time.time())
        pos_msg.data.append(deg2rad(value))
        self.pos_pub.publish(pos_msg)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'kp':
                self.pid.Kp = float(param.value)
            if param.name == 'ki':
                self.pid.Ki = float(param.value)
            if param.name == 'kd':
                self.pid.Kd = float(param.value)
        self.get_logger().info("update parameter")
        return SetParametersResult(successful=True)
    
    def imu_handler(self, msg: Imu):
        r, p, y = euler_from_quaternion(msg.orientation)
        effort = self.pid.update(rad2deg(p))
        self.get_logger().info(f"{rad2deg(p)} , {self.pid.last_error}")
        effort_msg = Float64MultiArray()
        effort_msg.data.append(effort)
        self.effort_pub.publish(effort_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
