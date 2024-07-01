# By Gerrit Dirk Lakerveld and Aqiel Oostenbrug (Jul 1, 2024)

# Libraries
import rclpy, sys, gpiozero, math, time
from astrolab.motor_node import MotorNode
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64
from simulation_interfaces.srv import Speed, Initialization  # type: ignore
from simulation_interfaces.action import Angle  # type: ignore

# Distance coordinates are given according to Triangular design
# To see how the points and thus the distances relate to oneanother please check drawings provided in ISBEP rapport of Gerrit Dirk Lakerveld 2024
# If linear motor is detached remeasure all distances and angles again to have a accurate set of Inverse Kinematics
# Constants
BD = 140
AG = 950
CG = 300
CD = 610
L0 = 600 # Unextended length of the linear actuator
AC = math.sqrt(AG**2 + CG**2)
gamma = math.asin(CG/AC)
alpha = math.radians(15.05)
deflection = 1
decoder_position= 1068.97
ENCODER_CONST = 1558
BUFFER_SIZE_ARM = 3
BUFFER_SIZE_TABLE = 2
BUFFER_DELTA = 1

# Motor direction constants
UP = 0
DOWN = 1
CLOCKWISE = 0
COUNTERCLOCKWISE = 1

# Arm Pins
arm_encoder = gpiozero.RotaryEncoder(14, 15, max_steps=0)
arm_pwm = gpiozero.PWMOutputDevice(12)
arm_dir = gpiozero.DigitalOutputDevice(25)
arm_switch = gpiozero.Button(5)

# Other constants
NAME = 'arm'
INITIAL_ANGLE = 0.0
INITIAL_SPEED = 0.3
QOS_PROFILE = 10
ANGLE_LOWER_BOUND = 0.0
ANGLE_UPPER_BOUND = 90.0
SPEED_LOWER_BOUND = 0.0
SPEED_UPPER_BOUND = 0.6

class ArmNode(MotorNode):
	""" Node for the arm of the Physical Sunlight Simulator. """

	def __init__(self):
		super().__init__(
			name=NAME,
			angle=INITIAL_ANGLE,
			speed=INITIAL_SPEED,
			qos_profile=QOS_PROFILE,
			angle_lower_bound=ANGLE_LOWER_BOUND,
			angle_upper_bound=ANGLE_UPPER_BOUND,
			speed_lower_bound=SPEED_LOWER_BOUND,
			speed_upper_bound=SPEED_UPPER_BOUND,
		)
		
	def angle_execute_callback(self, goal_handle):
		"""Provides responses and feedback for angle adjustment actions"""
		self.get_logger().info(
			f"[Action] Received angle adjustment request = {str(self.angle)} -> {str(goal_handle.request.requested_angle)}"
		)
		if (
			goal_handle.request.requested_angle >= self.angle_lower_bound
			and goal_handle.request.requested_angle <= self.angle_upper_bound
		):  # In valid range
			self.get_logger().info(
				f"[Action] Accepted angle adjustment request = {str(goal_handle.request.requested_angle)}"
			)
			# Calculating angles
			theta = math.radians(goal_handle.request.requested_angle)
			AD = math.sqrt(AC**2 + CD**2 - 2*AC*CD*math.cos(alpha + theta + gamma))
			AB = math.sqrt(AD**2 + BD**2)
			motor_extension = AB-L0
			arm_steps = round((decoder_position/deflection)*motor_extension)

			# Initializing condition variables
			feedback_msg = Angle.Feedback()
			feedback_msg.current_angle = self.angle

			try:
				# Direction and speed arm 
				if self.angle > goal_handle.request.requested_angle:
					arm_dir.value = DOWN
				elif self.angle < goal_handle.request.requested_angle:
					arm_dir.value = UP
				else:
					return Angle.Result()
					
				arm_pwm.value = self.speed
				
				#main while loop
				while arm_steps > abs(arm_encoder.steps):
					# Check if goal is still active
					if not goal_handle.is_active:
						self.get_logger().info(f"[Action] Angle adjustment goal got aborted = {str(goal_handle.request.requested_angle)}")
						arm_pwm.value = 0.0
						return Angle.Result()
					
					# Check if goal got cancel
					if goal_handle.is_cancel_requested:
						goal_handle.canceled()
						self.get_logger().info(f"[Action] Angle adjustment goal canceled = {str(goal_handle.request.requested_angle)}")
						arm_pwm.value = 0.0
						return Angle.Result()
						
					if arm_steps <= abs(arm_encoder.steps):
						arm_pwm.value = 0.0
						
					self.angle = math.degrees(abs(math.acos((BD**2 + AG**2 + CG**2 + CD**2-(abs(arm_encoder.steps)*deflection/decoder_position + L0)**2)/(2*AC*CD)) - alpha - gamma))
						
					# Publish feedback
					feedback_msg.current_angle = self.angle
					goal_handle.publish_feedback(feedback_msg)

					# Publish angle update
					self.update_angle_topic()
					self.get_logger().info(f"[Action] Current angle = {self.angle}")
			except:
				pass

			with self.goal_lock:
				if not goal_handle.is_active:
					self.get_logger().info(f"[Action] Angle adjustment goal got aborted = {str(goal_handle.request.requested_angle)}")
					arm_pwm.value = 0.0
					return Angle.Result()

			goal_handle.succeed()
		else:
			self.get_logger().info(
				f"[Action] Rejected angle adjustment request = {str(goal_handle.request.requested_angle)}"
			)
			goal_handle.abort()
		arm_pwm.value = 0.0
		# Publish angle update
		self.update_angle_topic()

		# Publish result
		result = Angle.Result()
		result.final_angle = self.angle

		return result
	
	def initialize_service_callback(self, request, response):
		"""Provides responses for initialize services"""
		self.get_logger().info(f"[Service] Received initialize request")

		with self.goal_lock:
		# This server only allows one goal at a time
			if self.goal_handle is not None and self.goal_handle.is_active:
				self.get_logger().info('[Service] Aborting previous goal')
				# Abort the existing goal
				self.goal_handle.abort()
			self.get_logger().info("[Service] Accepted initialize request")
			arm_dir.value = UP
			arm_pwm.value = 0.4
			time.sleep(5)
			arm_dir.value = DOWN
			arm_pwm.value = 0.3
			buffer_arm = BUFFER_SIZE_ARM
			while buffer_arm > 0:
					time.sleep(0.2)
					arm_switch_value = int(not arm_switch.value)
					if arm_switch_value == 0:
							buffer_arm = BUFFER_SIZE_ARM
					else:
							buffer_arm -= BUFFER_DELTA
					if buffer_arm == 0:
							arm_pwm.value = 0.0
			arm_encoder.steps = 0
			self.angle = 0.0
			response.response = True
		return response

def main():
	""" First function to be executed. """
	# Initialize rclpy
	rclpy.init(args=None)
	
	# Node control flow
	try:
		# Start node
		node = ArmNode()
		rclpy.spin(node, executor=MultiThreadedExecutor())
	except KeyboardInterrupt:
		# Ignore
		pass
	except ExternalShutdownException:
		# Graceful termination
		node.destroy_node()
		rclpy.shutdown()
		sys.exit(1)

if __name__ == "__main__":
	main()
