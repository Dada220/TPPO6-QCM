from robot import Robot
import time

time.sleep(4)
robot = Robot()
time.sleep(10)
robot.turn_left()
robot.turn_left()
robot.move_forward()
time.sleep(1)
robot.turn_left()
robot.move_backward()
time.sleep(4)
robot.destroy()
