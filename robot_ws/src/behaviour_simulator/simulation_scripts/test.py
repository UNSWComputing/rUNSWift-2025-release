# Add robots to the simulator with initial velocities
robot1 = Robot(player_number=1, initial_position=(0.0, 0.0))
simulator.add_robot(robot1)
robot2 = Robot(player_number=2, initial_position=(1000.0, -2000.0), initial_heading=1.5707963268)
simulator.add_robot(robot2)

# Set the ball with an initial velocity
ball = Ball(initial_position=(1000, 0), initial_velocity=(0, 300))
simulator.set_ball(ball)

