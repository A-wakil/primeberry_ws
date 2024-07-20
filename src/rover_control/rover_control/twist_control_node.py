import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
from Adafruit_MotorHAT import Adafruit_MotorHAT
import time
# import pigpio
import serial

class RoverControlNode(Node):

    def __init__(self):
        super().__init__('rover_control_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.control_callback,
            10
        )
        self.setup_rover()

    def setup_rover(self):
        try:
            self.mh = Adafruit_MotorHAT()
            self.motor_id = 1
            self.motor2_id = 2
            # self.servo_pin = 6
            # pigpio.pi()
            # pigpio.set_mode(self.servo_pin, pigpio.OUT)

            self.motor = self.mh.getMotor(self.motor_id)
            self.motor2 = self.mh.getMotor(self.motor2_id)

            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.send_serial_command("135")
            # GPIO.setmode(GPIO.BCM)
            # GPIO.setup(self.servo_pin, GPIO.OUT)

            self.speed = 200
            self.direction = Adafruit_MotorHAT.FORWARD
            # self.pwm = GPIO.PWM(self.servo_pin, 50)
            # self.pwm.start(7)
        
        except Exception as e:
            self.get_logger().error(f"Error setting up rover: {e}")

    def control_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        self.get_logger().info(f"Received velocities - Linear: {linear_velocity}, Angular: {angular_velocity}")
        try:
            if linear_velocity > 0:
                self.move_forward(min(linear_velocity, 1))
            elif linear_velocity < 0:
                self.move_backward(min(abs(linear_velocity), 1))
            else:
                self.stop_rover()

            if angular_velocity > 0:
                self.turn_left(min(angular_velocity, 1))
            elif angular_velocity < 0:
                self.turn_right(min(abs(angular_velocity), 1))
            else:
                self.send_serial_command(f"{135}")
            # else:
            #     self.stop_rover()
        except Exception as e:
            self.get_logger().error(f"Error executing command with velocities - Linear: {linear_velocity}, Angular: {angular_velocity}: {e}")
            self.stop_rover()

    def send_serial_command(self, command):
        if self.serial_port.is_open:
            self.serial_port.write(f"{command}\n".encode('utf-8'))

    def move_forward(self, speed):
        self.motor.setSpeed(int(speed * 255))  # Assuming speed is a fraction of max speed
        self.motor2.setSpeed(int(speed * 255))
        self.motor.run(Adafruit_MotorHAT.FORWARD)
        self.motor2.run(Adafruit_MotorHAT.FORWARD)

    def move_backward(self, speed):
        self.motor.setSpeed(int(speed * 255))  # Assuming speed is a fraction of max speed
        self.motor2.setSpeed(int(speed * 255))
        self.motor.run(Adafruit_MotorHAT.BACKWARD)
        self.motor2.run(Adafruit_MotorHAT.BACKWARD)

    def turn_left(self, angular_velocity):
        duty_cycle = 135 - ((angular_velocity / 1) * 55)  # Scale angular_velocity to range
        self.send_serial_command(f"{duty_cycle}")
        # self.pwm.ChangeDutyCycle(duty_cycle)
        # time.sleep(1)
        # self.pwm.ChangeDutyCycle(7)

    def turn_right(self, angular_velocity):
        duty_cycle = 135 + ((angular_velocity / 1) * 55)  # Scale angular_velocity to range
        self.send_serial_command(f"{duty_cycle}")
        # self.pwm.ChangeDutyCycle(duty_cycle)
        # time.sleep(1)
        # self.pwm.ChangeDutyCycle(7)

    def stop_rover(self):
        self.motor.run(Adafruit_MotorHAT.RELEASE)
        self.motor2.run(Adafruit_MotorHAT.RELEASE)
        self.send_serial_command(f"{135}")
        # self.pwm.stop()
        # GPIO.cleanup()

    def destroy_node(self):
        self.stop_rover()
        if self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RoverControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
