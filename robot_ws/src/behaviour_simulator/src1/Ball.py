import math


class Ball:
    RADIUS = 100  # Ball radius (used for collisions)

    def __init__(self, initial_position=(0.0, 0.0), initial_velocity=(0.0, 0.0)):
        self.position = list(initial_position)  # [x, y]
        self.velocity = list(initial_velocity)  # [vx, vy]
        self.friction = 0.5

    def tick(self, delta_time):
        self.position[0] += self.velocity[0] * delta_time
        self.position[1] += self.velocity[1] * delta_time

        decay_factor = max(0, 1 - self.friction * delta_time)
        self.velocity[0] *= decay_factor
        self.velocity[1] *= decay_factor

        if abs(self.velocity[0]) < 1:
            self.velocity[0] = 0.0
        if abs(self.velocity[1]) < 1:
            self.velocity[1] = 0.0

    def kick(self, force, direction):
        """
        Apply a force to the ball, simulating a kick.
        :param force: Magnitude of the force applied.
        :param direction: Direction of the kick (in radians).
        """
        self.velocity[0] += force * math.cos(direction)
        self.velocity[1] += force * math.sin(direction)
