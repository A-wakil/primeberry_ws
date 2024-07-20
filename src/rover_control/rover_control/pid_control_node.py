# ~/ros2_ws/src/rover_control/rover_control/pid_control_node.py

from Adafruit_MotorHAT import Adafruit_MotorHAT
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
import time

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

class PIDControlNode(Node):
    def __init__(self):
        super().__init__('pid_control_node')
        self.subscription = self.create_subscription(
            Int64MultiArray,
            'encoder_values',
            self.listener_callback,
            10)
        self.mh = Adafruit_MotorHAT()
        self.motor_id = 1
        self.motor2_id = 2
        self.motor = self.mh.getMotor(self.motor_id)
        self.motor2 = self.mh.getMotor(self.motor2_id)
        self.pid = PID(1.0, 0, 0)
        self.pid2 = PID(1.0, 0, 0)
        self.target_speed = 4500  # Setpoint for target speed in encoder counts per second
        self.max_encoder_speed = 5150  # Calculated max speed in encoder counts per second
        self.last_time = time.time()
        self.last_encoder1_value = 0
        self.last_encoder2_value = 0

    def listener_callback(self, msg):
        encoder1_value, encoder2_value = msg.data
        current_time = time.time()
        dt = current_time - self.last_time
        if dt > 0.01:

            # Calculate the difference in encoder values
            delta_encoder1 = encoder1_value - self.last_encoder1_value
            delta_encoder2 = encoder2_value - self.last_encoder2_value

            # Update last encoder values and time
            self.last_encoder1_value = encoder1_value
            self.last_encoder2_value = encoder2_value
            self.last_time = current_time

            # Calculate speed as change in encoder counts over time
            speed = delta_encoder1 / dt
            speed2 = delta_encoder2 / dt

            # Compute the control signals
            control = self.pid.compute(self.target_speed, speed)
            control2 = self.pid2.compute(self.target_speed, speed2)
            max_speed = 255

            motor_speed = (speed/self.max_encoder_speed)*max_speed
            motor2_speed = (speed2/self.max_encoder_speed)*max_speed
            # Scale the control signal to speed values (0 to 255)
            speed_control = min(max(int((control / self.max_encoder_speed) * max_speed), 0), max_speed)
            speed_control2 = min(max(int((control2 / self.max_encoder_speed) * max_speed), 0), max_speed)

            # Set motor speeds
            self.motor.setSpeed(speed_control+int(motor_speed))
            self.motor2.setSpeed(speed_control2+int(motor2_speed))

            # Run motors (assuming forward direction)
            # self.motor.run(Adafruit_MotorHAT.FORWARD)
            # self.motor2.run(Adafruit_MotorHAT.FORWARD)

            # Log the results
            self.get_logger().info(f'Motor1 Speed: {speed} Control: {control} Scaled: {speed_control}')
            self.get_logger().info(f'Motor2 Speed: {speed2} Control: {control2} Scaled: {speed_control2}')


def main(args=None):
    rclpy.init(args=args)
    node = PIDControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
