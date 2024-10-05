import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from inputs import get_key

class SwerveJoystick(Node):
    def __init__(self):
        super().__init__('swerve_joystick')
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.key_state = {'w': False, 's': False, 'a': False, 'd': False, 'left': False, 'right': False}

    def timer_callback(self):
        twist = Twist()
        self.process_inputs()

        # WASD for linear movement
        if self.key_state['w']:
            twist.linear.x = 1.0
        elif self.key_state['s']:
            twist.linear.x = -1.0
        else:
            twist.linear.x = 0.0

        if self.key_state['a']:
            twist.linear.y = 1.0
        elif self.key_state['d']:
            twist.linear.y = -1.0
        else:
            twist.linear.y = 0.0

        # Arrow keys for rotation
        if self.key_state['left']:
            twist.angular.z = 1.0
        elif self.key_state['right']:
            twist.angular.z = -1.0
        else:
            twist.angular.z = 0.0

        self.pub_cmd_vel.publish(twist)

    def process_inputs(self):
        events = get_key()
        for event in events:
            if event.ev_type == 'Key':
                if event.ev_type == 'Key' and event.state in (0, 1):  # Only process key press and release
                    key = event.code.lower()
                    is_pressed = event.state == 1
                    if key in self.key_state:
                        self.key_state[key] = is_pressed

def main(args=None):
    rclpy.init(args=args)
    joystick = SwerveJoystick()
    rclpy.spin(joystick)
    joystick.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()