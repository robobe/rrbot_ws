import rclpy
from rclpy.node import Node
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64MultiArray
from time import time
from sensor_msgs.msg import JointState

class PIDController:
    def __init__(self, p, i, d, i_min, i_max):
        self.kp = p
        self.ki = i
        self.kd = d
        self.i_min = i_min
        self.i_max = i_max
        self.prev_error = 0
        self.integral = 0

    def compute_command(self, error, dt):
        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error * dt
        # Clamp integral to prevent windup
        self.integral = max(self.i_min, min(self.integral, self.i_max))
        i_term = self.ki * self.integral

        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt
        self.prev_error = error

        # Calculate total output
        return p_term + i_term + d_term



class PositionEffortController(Node):
    def __init__(self):
        super().__init__('position_effort_controller')

        # Load PID parameters
        self.pid = PIDController(100.0, 0.01, 10.0, -100.0, 100.0)
        self.last_time = time()
        # Desired position (setpoint)
        self.desired_position = 1.5

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.state_callback, 10)
        self.create_subscription(Float64MultiArray, '/desired_position', self.setpoint_callback, 10)

        # Publisher to send effort command
        self.effort_pub = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)

    def state_callback(self, msg):
        # Get the current position of the joint
        current_position = msg.position[1]
        # Compute the position error
        error = self.desired_position - current_position

        # Calculate the effort using the PID controller
        current_time = time()
        dt = current_time - self.last_time
        effort = self.pid.compute_command(error, dt)
        self.last_time = current_time
        print(effort)
        # Publish the effort to the effort controller
        effort_msg = Float64MultiArray()
        effort_msg.data = [0.0, effort]  # For one joint
        self.effort_pub.publish(effort_msg)

    def setpoint_callback(self, msg):
        # Update the desired position (setpoint)
        self.desired_position = msg.data[0]

def main(args=None):
    rclpy.init(args=args)
    node = PositionEffortController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
