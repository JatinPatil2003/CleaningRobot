#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Float64MultiArray, Int32, Int64MultiArray
from geometry_msgs.msg import Twist


class PIDController:
   """
   A PID (Proportional-Integral-Derivative) controller class that calculates the appropriate output value based on the given feedback and predefined constants.
   """
   def __init__(
       self,
       kp,
       ki,
#Uses the provided proportional (kp), integral (ki), and derivative (kd) constants, setpoint, and optional parameters to initialize the PID controller.
       kd,
       setpoint,
       sample_time=10,
       output_limits=None,
       proportional_on_error=True,
   ):
       self.kp = kp
       self.ki = ki
       self.kd = kd
       self.setpoint = setpoint
       self.sample_time = sample_time
       self.proportional_on_error = proportional_on_error

       self.prev_input = 0
       self.output_sum = 0
       self.last_time = time.time()

       self.output_min, self.output_max = -255, 255

   def set_const(self, Kp, Ki, Kd, Setpoint):
       """
       Sets new PID constants and setpoint values.
       """
       self.kp = Kp
       self.ki = Ki
       self.kd = Kd
       self.setpoint = Setpoint

   def compute(self, feedback):
       """
       Calculates the PID output based on the given feedback value, elapsed time, and PID constants.
       """
       current_time = time.time()
       elapsed_time = current_time - self.last_time

       input_val = feedback
       error = self.setpoint - input_val
       d_input = input_val - self.prev_input

       self.output_sum += self.ki * error

       # Add Proportional on Measurement if P_ON_E is not specified
       if not self.proportional_on_error:
           self.output_sum -= self.kp * d_input

       if self.output_sum > self.output_max:
           self.output_sum = self.output_max
       elif self.output_sum < self.output_min:
           self.output_sum = self.output_min

       # Add Proportional on Error if P_ON_E is specified
       output = self.kp * error if self.proportional_on_error else 0

       # Compute the rest of PID output
       output += self.output_sum - self.kd * d_input

       if output > self.output_max:
           output = self.output_max
       elif output < self.output_min:
           output = self.output_min

       # Update variables for the next iteration
       self.prev_input = input_val
       self.last_time = current_time

       return output

class MotorFeedbackListener():
    def __init__(self):
        rospy.init_node("motor_controller")
        self.feedback_sub = rospy.Subscriber(
            "/motor/feedback", Int64MultiArray, self.feedback_callback, queue_size=10
        )
        self.cmd_vel_sub = rospy.Subscriber(
            "/cmd_vel", Twist, self.cmd_vel_callback, queue_size=10
        )
        self.left_pub = rospy.Publisher(
            "/motor/left_cmd", Int32, queue_size=10
        )
        self.right_pub = rospy.Publisher(
            "/motor/right_cmd", Int32, queue_size=10
        )
        self.last_time = time.time()
        self.last_left_counts = 0
        self.last_right_counts = 0
        self.total_counts_per_revolution = 3133
        self.left_motor_rpm = 0
        self.right_motor_rpm = 0
        self.cmd_left_motor_rpm = 0
        self.cmd_right_motor_rpm = 0
        self.left_motor_cmd = 0.0
        self.right_motor_cmd = 0.0

        self.wheel_separation = 0.288  # Adjust this value according to your robot's wheel separation
        self.wheel_radius = 0.055

    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        self.left_motor_cmd = (linear_velocity - angular_velocity * (self.wheel_separation / 2)) / (self.wheel_radius * 0.10472)
        self.right_motor_cmd = (linear_velocity + angular_velocity * (self.wheel_separation / 2)) / (self.wheel_radius * 0.10472)
        print(self.left_motor_cmd, self.right_motor_cmd)

        
    def get_right_motor_cmd(self):
        return self.right_motor_cmd

    def get_left_motor_cmd(self):
        return self.left_motor_cmd

    def feedback_callback(self, msg):
        if len(msg.data) >= 2:
            current_time = time.time()
            elapsed_time = current_time - self.last_time

            left_counts = msg.data[0]
            right_counts = msg.data[1]

            left_counts_delta = left_counts - self.last_left_counts
            right_counts_delta = right_counts - self.last_right_counts

            self.left_motor_rpm = self.calculate_rpm(left_counts_delta, elapsed_time)
            self.right_motor_rpm = self.calculate_rpm(right_counts_delta, elapsed_time)

            self.last_time = current_time
            self.last_left_counts = left_counts
            self.last_right_counts = right_counts
    
    def cmd_callback(self):
        left = 0
        right = 0
        if self.cmd_left_motor_pwm > 255:
            left = 255
        elif self.cmd_left_motor_pwm < -255:
            left = -255
        else:
            left = int(self.cmd_left_motor_pwm)

        if self.cmd_right_motor_pwm > 255:
            right = 255
        elif self.cmd_right_motor_pwm < -255:
            right = -255
        else:
            right = int(self.cmd_right_motor_pwm)

        self.left_pub.publish(left)
        self.right_pub.publish(right)
    
    def get_left_motor_rpm(self):
        return self.left_motor_rpm

    def get_right_motor_rpm(self):
        return self.right_motor_rpm
    
    def set_left_motor_rpm(self, cmd_pwm):
        self.cmd_left_motor_pwm = cmd_pwm

    def set_right_motor_rpm(self, cmd_pwm):
        self.cmd_right_motor_pwm = cmd_pwm

    def calculate_rpm(self, counts_delta, elapsed_time):
        counts_per_revolution = self.total_counts_per_revolution
        rpm = (counts_delta / counts_per_revolution) / (elapsed_time / 60.0)
        return rpm

# Initialize left and right PID controllers with given constants and setpoint
left_pid = None
right_pid = None
kp = 1.2
ki = 0.8
kd = 0.25
setpoint = 0.0

def main():
    global left_pid, right_pid
    motor_feedback_listener = MotorFeedbackListener()
    left_pid = PIDController(kp, ki, kd, setpoint)
    right_pid = PIDController(kp, ki, kd, setpoint)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        left_pid.setpoint = motor_feedback_listener.get_left_motor_cmd()
        right_pid.setpoint = motor_feedback_listener.get_right_motor_cmd()
        left = left_pid.compute(motor_feedback_listener.get_left_motor_rpm())
        right = right_pid.compute(motor_feedback_listener.get_right_motor_rpm())
        print(round(left_pid.setpoint,1),round(motor_feedback_listener.left_motor_rpm,1), round(right_pid.setpoint,1),round(motor_feedback_listener.right_motor_rpm,1))
        motor_feedback_listener.set_left_motor_rpm(left)
        motor_feedback_listener.set_right_motor_rpm(right)
        motor_feedback_listener.cmd_callback()

        rate.sleep()

if __name__ == "__main__":
    main()