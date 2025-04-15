import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import lgpio
import atexit

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # GPIO ピン定義
        self.LEFT_PWM = 18
        self.LEFT_IN1 = 23
        self.LEFT_IN2 = 24
        self.RIGHT_PWM = 13
        self.RIGHT_IN1 = 5
        self.RIGHT_IN2 = 6

        # GPIOハンドル（lgpio は open してから使う）
        self.h = lgpio.gpiochip_open(0)

        # ピンを出力に設定
        for pin in [self.LEFT_IN1, self.LEFT_IN2, self.RIGHT_IN1, self.RIGHT_IN2]:
            lgpio.gpio_claim_output(self.h, pin, 0)

        # PWM の初期化（duty 0 で停止）
        lgpio.tx_pwm(self.h, self.LEFT_PWM, 1000, 0)
        lgpio.tx_pwm(self.h, self.RIGHT_PWM, 1000, 0)

        # Joyトピック購読
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info('Motor Driver Node Started.')

        # シャットダウン時のGPIOリリース
        atexit.register(self.cleanup)

    def joy_callback(self, msg: Joy):
        linear = msg.axes[1]  # 前後
        turn = msg.axes[0]    # 左右

        # モータ速度計算
        left_speed = max(min((linear + turn) * 100, 100), -100)
        right_speed = max(min((linear - turn) * 100, 100), -100)

        self.set_motor_speed(left_speed, right_speed)

    def set_motor_speed(self, left, right):
        # 左モータの回転方向
        lgpio.gpio_write(self.h, self.LEFT_IN1, int(left >= 0))
        lgpio.gpio_write(self.h, self.LEFT_IN2, int(left < 0))
        lgpio.tx_pwm(self.h, self.LEFT_PWM, 1000, abs(left))

        # 右モータの回転方向
        lgpio.gpio_write(self.h, self.RIGHT_IN1, int(right >= 0))
        lgpio.gpio_write(self.h, self.RIGHT_IN2, int(right < 0))
        lgpio.tx_pwm(self.h, self.RIGHT_PWM, 1000, abs(right))

    def cleanup(self):
        # PWM 停止 & GPIO 解放
        lgpio.tx_pwm(self.h, self.LEFT_PWM, 1000, 0)
        lgpio.tx_pwm(self.h, self.RIGHT_PWM, 1000, 0)
        lgpio.gpiochip_close(self.h)


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

