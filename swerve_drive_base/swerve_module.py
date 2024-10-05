import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from constants import ModuleConstants, DriveConstants
from simple_pid import PID
from math import pi
import json

class SwerveModule(Node):
    def __init__(self, module_id):
        super().__init__('swerve_module_' + str(module_id))
        self.module_id = module_id
        self.sub_state = self.create_subscription(String, 'module_' + str(module_id) + '/desired_state', self.state_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.velocity = 0.0
        self.angle = 0.0
        self.drive_position = 0.0
        self.turning_position = 0.0
        self.turning_pid_controller = PID(ModuleConstants.kPTurning, 0, 0, setpoint=0)
        self.turning_pid_controller.sample_time = 0.02
        self.turning_pid_controller.output_limits = (-pi, pi)

    def timer_callback(self):
        self.get_logger().info(f'Module {self.module_id}: Velocity: {self.velocity}, Angle: {self.angle}, Drive Position: {self.drive_position}, Turning Position: {self.turning_position}')

    def state_callback(self, msg):
        try:
            state = json.loads(msg.data)
            self.set_desired_state(state['speed'], state['angle'])

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse state message: {e}")

    def set_desired_state(self, speed, angle):
        if abs(speed) < 0.001:
            self.stop()
            return

        optimized_angle = self.optimize_angle(angle)
        # TODO: Add motor control logic
        # Example: self.drive_motor.set(speed / DriveConstants.kPhysicalMaxSpeedMetersPerSecond)

        pid_output = self.turning_pid_controller(optimized_angle)
        # TODO: Add motor control logic
        # Example: self.turning_motor.set(pid_output)
        self.get_logger().info(f'Swerve[{self.module_id}] state: speed={speed}, angle={optimized_angle}')

    def optimize_angle(self, desired_angle):
        current_angle = self.get_turning_position()
        optimized_angle = desired_angle

        current_angle = self.normalize_angle(current_angle)
        optimized_angle = self.normalize_angle(optimized_angle)

        # Optimize based on minimizing angular displacement
        angle_diff = optimized_angle - current_angle
        if angle_diff > pi:
            optimized_angle -= 2 * pi
        elif angle_diff < -pi:
            optimized_angle += 2 * pi

        return optimized_angle

    def normalize_angle(self, angle):
        # Clamps to -pi, pi
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def get_drive_position(self):
        return self.drive_position  # TODO: Implement encoder reading

    def get_turning_position(self):
        return self.turning_position  # TODO: Implement encoder reading

    def stop(self):
        return # TODO STOP motors here

def main(args=None):
    rclpy.init(args=args)
    module = SwerveModule(1)
    rclpy.spin(module)
    module.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
