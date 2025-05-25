from robot import Robot
import time

# Команды движения
l = lambda r: r.turn_left()
r = lambda r: r.turn_right()
f = lambda r: r.move_forward()
b = lambda r: r.move_backward()
# Инициализация робота для управления
robot = Robot()
# Набор команд
commands = [f,f,f,f,l,l,l,l,l,l,b,b,b,b,b,b,b,b,r,r,r,r,r,r,r]
# Исполнение команд
for command in commands:
    command(robot)
    time.sleep(1)
# Уничтожение робота
robot.destroy()
