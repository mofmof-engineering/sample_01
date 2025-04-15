import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # GPIOチップオープン
        self.gpio = lgpio.gpiochip_open(0)

        # GPIOピン定義
        self.LEFT_PWM = 18
        self.LEFT_IN1 = 23
        self.LEFT_IN2 = 24
        self.RIGHT_PWM = 13
        self.RIGHT_IN1 = 5
        self.RIGHT_IN2 = 6

        # 出力として設定
        for pin in [self.LEFT_PWM, self.LEFT_IN1, self.LEFT_IN2,
                    self.RIGHT_PWM, self.RIGHT_IN1, self.RIGHT_IN2]:
            lgpio.gpio_claim_output(self.gpio, pin)

        # cmd_vel購読
        self.subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # 簡易差動計算
        left_speed = linear - angular
        right_speed = linear + angular

        self.set_motor(self.LEFT_PWM, self.LEFT_IN1, self.LEFT_IN2, left_speed)
        self.set_motor(self.RIGHT_PWM, self.RIGHT_IN1, self.RIGHT_IN2, right_speed)

    def set_motor(self, pwm_pin, in1_pin, in2_pin, speed):
        duty_cycle = min(max(abs(speed) * 100, 0), 100)  # 0〜100%
        direction = 1 if speed >= 0 else -1

        # INピン制御（前進 or 後退）
        lgpio.gpio_write(self.gpio, in1_pin, 1 if direction == 1 else 0)
        lgpio.gpio_write(self.gpio, in2_pin, 0 if direction == 1 else 1)

        # PWM出力（1kHz）
        lgpio.tx_pwm(self.gpio, pwm_pin, 1000, int(duty_cycle))

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    lgpio.gpiochip_close(node.gpio)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

