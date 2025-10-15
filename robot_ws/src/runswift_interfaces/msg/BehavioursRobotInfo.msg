uint8 ATTACKER=1
uint8 MIDFIELDER=2
uint8 DEFENDER=3
uint8 GOALIE=4

int32 player_number     # Player number of the robot
float32 robot_pos_x     # X-coordinate of the robot
float32 robot_pos_y     # Y-coordinate of the robot
float32 heading         # Heading of the robot (radians)

float32 ball_pos_x      # X-coordinate of the ball (absolute)
float32 ball_pos_y      # Y-coordinate of the ball (absolute)
float32 confidence      # Confidence score of the ball
builtin_interfaces/Time timestamp

bool kick_success       # True if kick has been completed
uint8 player_role       # Role of the robot
bool my_ball            # robot has possession of the ball
bool kicking_team       # True if we are team kicking it in
uint8 ref_detected      # True if ref detected

