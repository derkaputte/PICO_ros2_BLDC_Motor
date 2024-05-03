import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('differential_drive_controller')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10)
        self.left_motor_pub = self.create_publisher(Int32, 'MOTOR_pwm', 10)
        self.right_motor_pub = self.create_publisher(Int32, 'MOTOR2_pwm', 10)
        self.wheel_distance = 0.35  # Abstand zwischen den Rädern in Metern
        self.max_pwm_value = 255  # Maximaler PWM-Wert für die Motoren

    def twist_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        left_speed = linear_x - angular_z * self.wheel_distance / 2
        right_speed = linear_x + angular_z * self.wheel_distance / 2

        left_pwm = self.speed_to_pwm(left_speed)
        right_pwm = self.speed_to_pwm(right_speed)

        left_pwm_msg = Int32()
        left_pwm_msg.data = left_pwm
        right_pwm_msg = Int32()
        right_pwm_msg.data = right_pwm

        self.left_motor_pub.publish(left_pwm_msg)
        self.right_motor_pub.publish(right_pwm_msg)

    def speed_to_pwm(self, speed):
        # Konvertiere Geschwindigkeit in PWM-Wert
        pwm = int(speed * self.max_pwm_value / 255)
        # Stelle sicher, dass der PWM-Wert im gültigen Bereich liegt
        pwm = max(-255, min(255, pwm))
        return pwm

def main(args=None):
    rclpy.init(args=args)
    differential_drive_controller = DifferentialDriveController()
    rclpy.spin(differential_drive_controller)
    differential_drive_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
