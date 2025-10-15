robot1 = Robot(player_number=1, initial_position=(0.0, 0.0), initial_heading=0.0)
simulator.add_robot(robot1)
# ball = Ball(initial_position=(300.0, 0.0)) # Left foot
ball = Ball(initial_position=(600.0, 0.0), initial_velocity=(-600, 0))  # Right foot
simulator.set_ball(ball)
