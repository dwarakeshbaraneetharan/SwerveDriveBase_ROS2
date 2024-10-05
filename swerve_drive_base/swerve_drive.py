import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from constants import DriveConstants
from math import hypot, atan2, pi
import json

class SwerveDrive(Node):
    def __init__(self):
        super().__init__('swerve_drive')
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.modules = [self.create_swerve_module(i) for i in range(4)]

    def create_swerve_module(self, module_id):
        state_pub = self.create_publisher(String, f'module_{module_id}/desired_state', 10)
        return {'state': state_pub}

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Received command: linear x={msg.linear.x} y={msg.linear.y} angular z={msg.angular.z}')
        desired_states = self.calculate_desired_states(msg.linear.x, msg.linear.y, msg.angular.z)
        self.desaturate_wheel_speeds(desired_states, DriveConstants.kPhysicalMaxSpeedMetersPerSecond)
        
        for i, state in enumerate(desired_states):
            state_message = json.dumps({'speed': state['speed'], 'angle': state['angle']})
            self.modules[i]['state'].publish(String(data=state_message))

    def calculate_desired_states(self, vx, vy, omega):
        L = DriveConstants.kWheelBase
        W = DriveConstants.kTrackWidth
        R = hypot(L, W)

        A = vx - omega * (L / R)
        B = vx + omega * (L / R)
        C = vy - omega * (W / R)
        D = vy + omega * (W / R)

        states = [
            {'speed': hypot(B, D), 'angle': atan2(B, D)},
            {'speed': hypot(B, C), 'angle': atan2(B, C)},
            {'speed': hypot(A, D), 'angle': atan2(A, D)},
            {'speed': hypot(A, C), 'angle': atan2(A, C)}
        ]
        return states

    def desaturate_wheel_speeds(self, states, max_speed):
        max_wheel_speed = max(state['speed'] for state in states)
        if max_wheel_speed > max_speed:
            scale_factor = max_speed / max_wheel_speed
            for state in states:
                state['speed'] *= scale_factor

def main(args=None):
    rclpy.init(args=args)
    drive = SwerveDrive()
    rclpy.spin(drive)
    drive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
