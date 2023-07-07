import time
import board
from adafruit_motorkit import MotorKit
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Drive(Node):
    def __init__(self):
        super().__init__('drive')
        self.sub = self.create_subscription(Twist, '/turtle1/cmd_vel', self.msg_callback, 10)

    def msg_callback(self, msg):
        print(f'{msg.linear.x}, {msg.angular.z}')
        x = msg.linear.x
        z = msg.angular.z
        if(x == 2):
            mechanum.forward(1.0)
            time.sleep(0.5)
            mechanum.stop()
        elif(x == -2):
            mechanum.forward(-1.0)
            time.sleep(0.5)
            mechanum.stop()
        if(z == 2):
            mechanum.rotate_ccw(1.0)
            time.sleep(0.5)
            mechanum.stop()
        elif(z == -2):
            mechanum.rotate_cw(1.0)
            time.sleep(0.5)
            mechanum.stop()

class MechanumDrive:
    def __init__(self):
        self.kit = MotorKit(i2c=board.I2C())
        self.stop()

    def setfr(self, power):
        self.kit.motor1.throttle = power

    def setfl(self, power):
        self.kit.motor2.throttle = power

    def setbr(self, power):
        self.kit.motor3.throttle = power

    def setbl(self, power):
        self.kit.motor4.throttle = power

    def forward(self, power):
        self.setfr(power)
        self.setfl(power)
        self.setbr(power)
        self.setbl(power)

    def backword(self, power):
        self.setfr(-power)
        self.setfl(-power)
        self.setbr(-power)
        self.setbl(-power)

    def rotate_cw(self, power):
        self.setfr(-power)
        self.setfl( power)
        self.setbr(-power)
        self.setbl( power)

    def rotate_ccw(self, power):
        self.setfr( power)
        self.setfl(-power)
        self.setbr( power)
        self.setbl(-power)

    def stop(self):
        self.setfr(0)
        self.setfl(0)
        self.setbr(0)
        self.setbl(0)

def main():
    global mechanum
    mechanum = MechanumDrive()
    mechanum.stop()

    rclpy.init()
    node = Drive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
