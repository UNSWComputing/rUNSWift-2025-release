import sys
import math
import pygame
import rclpy
from rclpy.node import Node
from runswift_interfaces.msg import SimulationState

# Constants for field dimensions (in mm)
FIELD_LENGTH = 9000
FIELD_WIDTH = 6000
CENTER_CIRCLE_DIAMETER = 1500
GOAL_AREA_WIDTH = 600
GOAL_AREA_HEIGHT = 2200
PENALTY_AREA_WIDTH = 1650
PENALTY_AREA_HEIGHT = 4000
PENALTY_MARK_DISTANCE = 1300  # Distance from goal to penalty mark (in mm)
PENALTY_MARK_SIZE = 100  # Penalty mark size (diameter, in mm)
CENTER_DOT_SIZE = 100  # Center dot size (diameter, in mm)
BALL_RADIUS = 100  # Ball size in mm
ROBOT_RADIUS = 200  # Robot size in mm

# Padding for the field (in pixels)
FIELD_PADDING = 50

# Pygame Colors
GREEN = (0, 128, 0)
WHITE = (255, 255, 255)
YELLOW = (255, 255, 0)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)
GRAY = (50, 50, 50)


class RendererNode(Node):
    def __init__(self):
        super().__init__("renderer_node")
        self.subscription = self.create_subscription(
            SimulationState, "/simulation/state", self.update_simulation_state, 10
        )
        self.simulation_state = None

        # Initialize Pygame
        pygame.init()
        self.window_width = 1400
        self.window_height = 800
        self.sidebar_width = 300
        self.field_width = self.window_width - self.sidebar_width

        self.screen = pygame.display.set_mode((self.window_width, self.window_height))
        pygame.display.set_caption("Simulation Renderer")
        self.clock = pygame.time.Clock()
        self.running = True

    def update_simulation_state(self, msg):
        """Callback to update the simulation state."""
        self.simulation_state = msg

    def draw_field(self):
        """Draw the soccer field."""
        # Fill the entire screen with green
        self.screen.fill(GREEN)

        # Calculate field drawing area with padding
        field_area = pygame.Rect(
            FIELD_PADDING, FIELD_PADDING, self.field_width - FIELD_PADDING * 2, self.window_height - FIELD_PADDING * 2
        )

        # Scale dimensions to fit the field area
        scale_x = field_area.width / FIELD_LENGTH
        scale_y = field_area.height / FIELD_WIDTH
        self.scale = min(scale_x, scale_y)

        # Draw field boundary
        field_rect = pygame.Rect(0, 0, FIELD_LENGTH * self.scale, FIELD_WIDTH * self.scale)
        field_rect.center = field_area.center
        pygame.draw.rect(self.screen, WHITE, field_rect, 5)

        # Draw center circle
        pygame.draw.circle(self.screen, WHITE, field_rect.center, (CENTER_CIRCLE_DIAMETER // 2) * self.scale, 5)

        # Draw center line
        pygame.draw.line(
            self.screen, WHITE, (field_rect.centerx, field_rect.top), (field_rect.centerx, field_rect.bottom), 5
        )

        # Left goal box
        left_goal_box = pygame.Rect(
            field_rect.left,
            field_rect.centery - GOAL_AREA_HEIGHT * self.scale / 2,
            GOAL_AREA_WIDTH * self.scale,
            GOAL_AREA_HEIGHT * self.scale,
        )
        pygame.draw.rect(self.screen, WHITE, left_goal_box, 5)

        left_penalty_box = pygame.Rect(
            field_rect.left,
            field_rect.centery - PENALTY_AREA_HEIGHT * self.scale / 2,
            PENALTY_AREA_WIDTH * self.scale,
            PENALTY_AREA_HEIGHT * self.scale,
        )
        pygame.draw.rect(self.screen, WHITE, left_penalty_box, 5)

        # Right goal box
        right_goal_box = pygame.Rect(
            field_rect.right - GOAL_AREA_WIDTH * self.scale,
            field_rect.centery - GOAL_AREA_HEIGHT * self.scale / 2,
            GOAL_AREA_WIDTH * self.scale,
            GOAL_AREA_HEIGHT * self.scale,
        )
        pygame.draw.rect(self.screen, WHITE, right_goal_box, 5)

        right_penalty_box = pygame.Rect(
            field_rect.right - PENALTY_AREA_WIDTH * self.scale,
            field_rect.centery - PENALTY_AREA_HEIGHT * self.scale / 2,
            PENALTY_AREA_WIDTH * self.scale,
            PENALTY_AREA_HEIGHT * self.scale,
        )
        pygame.draw.rect(self.screen, WHITE, right_penalty_box, 5)

        # Draw penalty dots
        penalty_mark_distance_scaled = PENALTY_MARK_DISTANCE * self.scale
        penalty_mark_size_scaled = PENALTY_MARK_SIZE * self.scale

        # Left penalty dot
        left_penalty_dot_x = field_rect.left + penalty_mark_distance_scaled
        left_penalty_dot_y = field_rect.centery
        pygame.draw.circle(
            self.screen, WHITE, (int(left_penalty_dot_x), int(left_penalty_dot_y)), int(penalty_mark_size_scaled // 2)
        )

        # Right penalty dot
        right_penalty_dot_x = field_rect.right - penalty_mark_distance_scaled
        right_penalty_dot_y = field_rect.centery
        pygame.draw.circle(
            self.screen, WHITE, (int(right_penalty_dot_x), int(right_penalty_dot_y)), int(penalty_mark_size_scaled // 2)
        )

        # Draw center dot
        center_dot_size_scaled = CENTER_DOT_SIZE * self.scale
        pygame.draw.circle(self.screen, WHITE, field_rect.center, int(center_dot_size_scaled // 2))

        # Save the field area center for positioning robots and balls
        self.field_center = field_rect.center

    def draw_sidebar(self):
        """Draw the sidebar to display robot information."""
        sidebar_area = pygame.Rect(self.field_width, 0, self.sidebar_width, self.window_height)
        pygame.draw.rect(self.screen, GRAY, sidebar_area)

        font = pygame.font.Font(None, 30)
        y_offset = 10

        if self.simulation_state:
            # Sidebar header
            header = font.render("Robot Info", True, WHITE)
            self.screen.blit(header, (self.field_width + 10, y_offset))
            y_offset += 40

            for robot in self.simulation_state.robots:
                robot_info = (
                    f"Player {robot.player_number}:\n"
                    f"Pos: ({robot.pos_x:.1f}, {robot.pos_y:.1f})\n"
                    f"Heading: {math.degrees(robot.heading):.1f}°"
                )
                for line in robot_info.split("\n"):
                    line_surface = font.render(line, True, WHITE)
                    self.screen.blit(line_surface, (self.field_width + 10, y_offset))
                    y_offset += 30

                y_offset += 20  # Add space between robots

    def draw_ball(self, ball_pos_x, ball_pos_y):
        """Draw the ball on the field."""
        ball_x = self.field_center[0] + ball_pos_x * self.scale
        ball_y = self.field_center[1] - ball_pos_y * self.scale
        pygame.draw.circle(self.screen, YELLOW, (int(ball_x), int(ball_y)), int(BALL_RADIUS * self.scale))

    def draw_robot(self, robot):
        """Draw a robot on the field."""
        robot_x = self.field_center[0] + robot.pos_x * self.scale
        robot_y = self.field_center[1] - robot.pos_y * self.scale
        pygame.draw.circle(self.screen, BLUE, (int(robot_x), int(robot_y)), int(ROBOT_RADIUS * self.scale))

        # Draw heading line
        heading_x = robot_x + math.cos(robot.heading) * ROBOT_RADIUS * self.scale
        heading_y = robot_y - math.sin(robot.heading) * ROBOT_RADIUS * self.scale
        pygame.draw.line(self.screen, WHITE, (robot_x, robot_y), (heading_x, heading_y), 2)

        # Draw semi-transparent cone for head direction
        self.draw_head_cone(robot_x, robot_y, robot.heading, robot.yaw)

        # Draw player number
        font = pygame.font.Font(None, 24)
        text_surface = font.render(str(robot.player_number), True, WHITE)
        self.screen.blit(text_surface, (robot_x - 10, robot_y - 10))

        # Draw perceived ball position if it doesn't match the true ball position
        if robot.ball_pos_x != self.simulation_state.ball_pos_x or robot.ball_pos_y != self.simulation_state.ball_pos_y:
            perceived_ball_x = self.field_center[0] + robot.ball_pos_x * self.scale
            perceived_ball_y = self.field_center[1] - robot.ball_pos_y * self.scale

            # Create a semi-transparent surface for the perceived ball
            transparent_ball = pygame.Surface(
                (int(BALL_RADIUS * self.scale * 2), int(BALL_RADIUS * self.scale * 2)), pygame.SRCALPHA
            )
            pygame.draw.circle(
                transparent_ball,
                (200, 200, 200, 128),  # Gray with 50% transparency
                (int(BALL_RADIUS * self.scale), int(BALL_RADIUS * self.scale)),
                int(BALL_RADIUS * self.scale),
            )
            self.screen.blit(
                transparent_ball,
                (perceived_ball_x - BALL_RADIUS * self.scale, perceived_ball_y - BALL_RADIUS * self.scale),
            )

            # Render player number next to perceived ball
            font = pygame.font.Font(None, 18)
            text_surface = font.render(str(robot.player_number), True, (50, 50, 50))
            self.screen.blit(text_surface, (perceived_ball_x - 4, perceived_ball_y - 7))

    def draw_head_cone(self, robot_x, robot_y, heading, yaw):
        """Draw a semi-transparent circular segment indicating the head's looking direction."""
        # Cone parameters
        cone_length = 400  # Length of the cone in mm
        cone_angle = math.radians(56.3)  # Angle of the cone in radians

        # Scale cone length
        cone_length_scaled = cone_length * self.scale

        # Calculate the center direction (heading + yaw)
        cone_center_angle = heading - yaw

        # Calculate left and right boundaries of the cone
        left_angle = cone_center_angle - cone_angle / 2
        right_angle = cone_center_angle + cone_angle / 2

        # Create a surface for transparency
        surface = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)

        # Draw the circular segment (arc)
        arc_center = (robot_x, robot_y)
        arc_rect = pygame.Rect(
            robot_x - cone_length_scaled,
            robot_y - cone_length_scaled,
            2 * cone_length_scaled,
            2 * cone_length_scaled,
        )
        pygame.draw.arc(
            surface,
            (255, 255, 255, 128),  # White with transparency
            arc_rect,
            left_angle,
            right_angle,
            width=int(cone_length_scaled * 0.05),  # Arc thickness
        )

        # Fill the segment area
        points = [(robot_x, robot_y)]
        num_points = 50  # Increase for smoother segment
        for i in range(num_points + 1):
            angle = left_angle + (right_angle - left_angle) * i / num_points
            x = robot_x + math.cos(angle) * cone_length_scaled
            y = robot_y - math.sin(angle) * cone_length_scaled
            points.append((x, y))

        pygame.draw.polygon(surface, (255, 255, 255, 64), points)
        self.screen.blit(surface, (0, 0))

    def run(self):
        """Main loop for the renderer."""
        while self.running:

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            self.draw_field()
            if self.simulation_state:

                # Draw robots
                for robot in self.simulation_state.robots:
                    self.draw_robot(robot)

                # Draw ball
                if self.simulation_state.ball_pos_x is not None:
                    self.draw_ball(self.simulation_state.ball_pos_x, self.simulation_state.ball_pos_y)

            # Draw sidebar
            self.draw_sidebar()

            pygame.display.flip()
            self.clock.tick(30)  # Refresh at 30 Hz

            rclpy.spin_once(self)

        pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    node = RendererNode()
    node.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
