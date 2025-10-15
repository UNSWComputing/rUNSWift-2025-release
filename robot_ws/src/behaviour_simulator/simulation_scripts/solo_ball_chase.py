robot1 = Robot(player_number=1, initial_position=(-1000.0, -3000.0), initial_heading=1.7)
simulator.add_robot(robot1)
ball = Ball(initial_position=(1000, -300), initial_velocity=(300, 0))
simulator.set_ball(ball)

