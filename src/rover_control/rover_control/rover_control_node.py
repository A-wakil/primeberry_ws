# ~/ros2_ws/src/rover_control/rover_control/rover_control_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
from Adafruit_MotorHAT import Adafruit_MotorHAT
import time
import os
import sys
# import pigpio
import serial

class RoverControlNode(Node):

    def __init__(self):
        super().__init__('rover_control_node')
        self.subscription = self.create_subscription(
            String,
            'rover_commands',
            self.control_callback,
            10
        )
        self.setup_rover()

    def setup_rover(self):
        try:
            self.mh = Adafruit_MotorHAT()
            self.motor_id = 1
            self.motor2_id = 2
            self.servo_pin = 18

            self.motor = self.mh.getMotor(self.motor_id)
            self.motor2 = self.mh.getMotor(self.motor2_id)

            # GPIO.setmode(GPIO.BCM)
            # GPIO.setup(self.servo_pin, GPIO.OUT)

            # self.pi = pigpio.pi() # Connect to local Pi.
            # if not self.pi.connected:
            #     raise RuntimeError("pigpio not connected")

            self.speed = 200
            self.direction = Adafruit_MotorHAT.FORWARD
            # self.pwm = GPIO.PWM(self.servo_pin, 50)
            # self.pwm.start(7)
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

            self.servo_neutral = 135
            self.servo_left = 80
            self.servo_right = 190

            # self.pi.set_mode(self.servo_pin, pigpio.OUTPUT)
            self.send_serial_command(f"{self.servo_neutral}")
            # self.pi.set_servo_pulsewidth(self.servo_pin, self.servo_neutral)
        
        except Exception as e:
            self.get_logger().error(f"Error setting up rover: {e}")

    def control_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: {command}")
        try:
            if command == "move_forward":
                self.move_forward()
            elif command == "turn_left":
                self.turn_left()
            elif command == "turn_right":
                self.turn_right()
            elif command == "stop":
                self.stop_rover()
            else:
                self.send_serial_command(f"{command}")
        except Exception as e:
            self.get_logger().error(f"Error executing command {command}: {e}")
            self.stop_rover()
    def send_serial_command(self, command):
        if self.serial_port.is_open:
            self.serial_port.write(f"{command}\n".encode('utf-8'))

    def move_forward(self):
        self.motor.setSpeed(self.speed)
        self.motor2.setSpeed(self.speed)
        self.motor.run(self.direction)
        self.motor2.run(self.direction)

    def turn_left(self):
        # self.pi.set_servo_pulsewidth(self.servo_pin, self.servo_left)
        self.send_serial_command(f"{self.servo_left}")
        time.sleep(1)
        self.send_serial_command(f"{self.servo_neutral}")
        # self.pi.set_servo_pulsewidth(self.servo_pin, self.servo_neutral)
        

    def turn_right(self):
        self.send_serial_command(f"{self.servo_right}")
        # self.pi.set_servo_pulsewidth(self.servo_pin, self.servo_right)
        time.sleep(1)
        # self.pi.set_servo_pulsewidth(self.servo_pin, self.servo_neutral)
        self.send_serial_command(f"{self.servo_neutral}")

    def stop_rover(self):
        self.motor.run(Adafruit_MotorHAT.RELEASE)
        self.motor2.run(Adafruit_MotorHAT.RELEASE)
        # self.pi.set_servo_pulsewidth(self.servo_pin, self.servo_neutral)
        self.send_serial_command(f"{self.servo_neutral}")

    def destroy_node(self):
        # self.pi.stop()
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
