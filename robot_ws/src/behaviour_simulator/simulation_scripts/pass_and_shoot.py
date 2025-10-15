robot1 = Robot(player_number=1, initial_position=(0.0, 0.0))
robot2 = Robot(player_number=2, initial_position=(1000.0, 0.0))
robot3 = Robot(player_number=3, initial_position=(2000.0, 0.0))
simulator.add_robot(robot1)
simulator.add_robot(robot2)
simulator.add_robot(robot3)
ball = Ball(initial_position=(500.0, 0.0))
simulator.set_ball(ball)

