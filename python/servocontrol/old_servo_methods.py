# def servo_movement_loop(
#     servo: ServoKit,
#     channel: int,
#     step_degrees: float,
#     debug: bool,
#     reverse: bool = False,
# ) -> None:
#     """
#     Control a steering servo using signed steering coordinates.

#     Logical steering:
#         -90 degrees = full left
#           0 degrees = centered
#         +90 degrees = full right

#     ServoKit command:
#           0 degrees = one servo endpoint
#          90 degrees = servo center
#         180 degrees = opposite endpoint
#     """

#     fd = sys.stdin.fileno()
#     original_term_settings = termios.tcgetattr(fd)

#     selected_servo = servo.servo[channel]

#     # This is the rover's steering angle, NOT ServoKit's absolute angle.
#     steering_angle = 0.0

#     def write_steering_angle(angle: float) -> None:
#         """
#         Convert signed rover steering angle into ServoKit's 0–180 range.
#         """
#         nonlocal steering_angle

#         steering_angle = max(-90.0, min(90.0, float(angle)))

#         if reverse:
#             servo_angle = 90.0 - steering_angle
#         else:
#             servo_angle = 90.0 + steering_angle

#         selected_servo.angle = servo_angle

#         if debug:
#             print(
#                 f"STEERING: {steering_angle:+.1f} deg | "
#                 f"SERVO COMMAND: {servo_angle:.1f} deg",
#                 end="\r\n",
#             )

#     try:
#         # Explicitly center it when the loop begins.
#         write_steering_angle(0.0)

#         if debug:
#             print(f"Controlling servo channel {channel}.")
#             print("A/D steer, SPACE centers, F disables, C exits.")

#         tty.setraw(fd)

#         while True:
#             movement = sys.stdin.read(1).lower()

#             if movement == "a":
#                 write_steering_angle(steering_angle - step_degrees)

#             elif movement == "d":
#                 write_steering_angle(steering_angle + step_degrees)

#             elif movement == " ":
#                 write_steering_angle(0.0)

#             elif movement == "f":
#                 selected_servo.angle = None

#                 # Do not reset steering_angle. Disabling PWM does not
#                 # magically move the shaft or establish a new position.
#                 if debug:
#                     print(
#                         "PWM DISABLED — stored steering command retained",
#                         end="\r\n",
#                     )

#             elif movement == "c":
#                 selected_servo.angle = None
#                 break

#     finally:
#         selected_servo.angle = None
#         termios.tcsetattr(
#             fd,
#             termios.TCSADRAIN,
#             original_term_settings,
#         )