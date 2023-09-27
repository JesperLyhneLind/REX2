# from statemachine import StateMachine, State
# from enum import Enum
# from time import sleep
# import robot

# class status(Enum):
#     STOP = 1
#     DRIVE = 2
#     TURN = 3
#     LOOK = 4

# class Direction(Enum):
#     Left = 1
#     Right = 2

# class Ostato(StateMachine):
#     "A state for our otto machine"
#     stop = State(initial=True)
#     drive = State()
#     turn = State()
#     look = State()
# drive(while checking)
# driveStraight
# driveToTarget
# sonarObstacleAvoidance
# camera
# turn
# stop

# class StateMachine:
#     global arlo
#     def __init__(self):
#         arlo = robot.Robot()
#         self.state = State.LOOK

#     def transition_to(self, new_state):
#         self.state = new_state
#         print(f"Transitioned to state: {self.state}")

#     def update(self, frameReference, tvecs):
#         if self.state == State.LOOK:
#             self.look(frameReference, tvecs)
#         elif self.state == State.TURN:
#             self.turn(frameReference, tvecs)
#         elif self.state == State.DRIVE:
#             self.drive(frameReference, tvecs)
#         elif self.state == State.STOP:
#             self.stop()
    
#     def look(self, frameReference, tvecs):
#         # Your existing LOOK logic goes here
#         # Example: Check conditions and transition to TURN or DRIVE
#         if tvecs is not None:
#             self.transition_to(State.TURN)
#         else:
#             self.transition_to(State.DRIVE)

#     def turn(self, dir: Direction, angle: int):
#         if dir == Direction.Left:
#             print(arlo.go_diff(49, 49, 0, 1))
#             sleep(angle/90)
#             print(arlo.stop())
#             sleep(0.041)
#         else:
#             print(arlo.go_diff(49, 49, 1, 0))
#             sleep(angle/90)
#             print(arlo.stop())
#             sleep(0.041)
#         # Your existing TURN logic goes here
#         # Example: Check conditions and transition to LOOK or DRIVE
#         self.transition_to(State.LOOK)

#     def go_to_box(self, angle_sign, angle, dist):
#         if angle_sign[0] == -1:
#             turn(self, Direction.Left, angle)
#             driveM((dist - 500) / 100) #drive to box with 50cm to spare
#         elif angle_sign[0] == 1:
#             turn(Direction.Right, angle)
#             driveM((dist - 500) / 100)
#         else:
#             driveM((dist - 500) / 100)

#     def driveM(meters):
#         leftSpeed = 70
#         rightSpeed = 70
#         print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
#         # Wait a bit while robot moves forward
#         sleep(1.7*meters)
#         # send a stop command
#         print(arlo.stop())   
#         sleep(0.041)  
#         self.transition_to(State.LOOK)

#     def stop(self):
#         # Your existing STOP logic goes here
#         # Example: Check conditions and transition to LOOK or another state
#         self.transition_to(State.LOOK)

#     def perform_action(self):
#         if self.state == 'idle':
#             print("idle...")
#         elif self.state == 'drive':
#             print("Driving...")
#         elif self.state == 'stopped':
#             print("Stopped...")
#         elif self.state == 'turn':
#             print("turning")
#         else:
#             print("Unknown state!")


# sm = StateMachine() 
# sm.drive()