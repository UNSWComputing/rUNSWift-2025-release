from src.datamodels.RobotPose import RobotPose
from src.utils.MathUtil import distancePointToPoint
from src.utils.Ball import getAbsoluteBall
from geometry_msgs.msg import Point
from runswift_interfaces.msg import SeBall
from rclpy.clock import Clock
from rclpy.time import Time

def getTeamBall(blackboard):
    """
    Finds the absolute ball coordinates with the highest confidence score out of all robots.

    Returns:
        a SeBall
    """
    max_confidence = float('-inf')
    team_info = blackboard.robot_comms
    best_ball = None
    clock = Clock()
    current_time = clock.now().to_msg().sec
    for robot in team_info.values():
        msg_time = robot.timestamp.sec
        if robot.confidence > max_confidence and (current_time - msg_time) < 3:
            best_ball = (robot.ball_pos_x, robot.ball_pos_y)
            max_confidence = robot.confidence
    msg = SeBall()
    if best_ball is not None:
        msg.confidence = max_confidence
        msg.pos_x = best_ball[0]
        msg.pos_y = best_ball[1]
    else:
        return None

    return msg

def getClosestRobotToPoint(blackboard, target_pos=RobotPose(), excluded_robots=[]):
    """
    Finds the player who is currently closest to the target_pos, excluding robots with the player number provided

    Returns:
        the player number associated with that robot
    """
    closest_distance = float('inf')
    closest_robot = None
    for robot in blackboard.robot_comms.values():
        # skip excluded robots
        if robot.player_number in excluded_robots:
            continue
        robot_pos = RobotPose(point=Point(x=robot.robot_pos_x, y=robot.robot_pos_y), heading=robot.heading)
        dist_between = distancePointToPoint(robot_pos, target_pos)
        if dist_between < closest_distance:
            closest_robot = robot.player_number
            closest_distance = dist_between
    if closest_robot is None:
        closest_robot = blackboard.player_info.player_number
    return closest_robot

def getClosestRobotToTeamBall(blackboard):
    """
    Finds the player who is the closest to our best estimate of the ball position.

    Returns:
    the player number associated with that robot
    """
    ball = getAbsoluteBall(blackboard)
    if ball is None:
        ball = getTeamBall(blackboard)
    if ball is None:
        return blackboard.player_info.player_number
    return getClosestRobotToPoint(blackboard, RobotPose(point=Point(x=ball.pos_x, y=ball.pos_y)))


def get_requirements(valid_players, isDefence):
  num_players = len(valid_players)
  if num_players == 7:
    return {"Attacker": 2, "Defender": 2, "Midfielder": 2, "Goalkeeper": 1}
  elif num_players == 6:
    return {"Attacker": 2, "Defender": 2, "Midfielder": 1, "Goalkeeper": 1}
  elif num_players == 5:
    return {"Attacker": 2, "Defender": 2, "Midfielder": 0, "Goalkeeper": 1}
  elif num_players == 4:
    return {"Attacker": 1, "Defender": 2, "Midfielder": 0, "Goalkeeper": 1} if isDefence else {"Attacker": 2, "Defender": 1, "Midfielder": 0, "Goalkeeper": 1}
  elif num_players == 3:
    return {"Attacker": 0, "Defender": 2, "Midfielder": 0, "Goalkeeper": 1} if isDefence else {"Attacker": 1, "Defender": 1, "Midfielder": 0, "Goalkeeper": 1}
  elif num_players == 2:
    return {"Attacker": 0, "Defender": 1, "Midfielder": 0, "Goalkeeper": 1} if isDefence else {"Attacker": 0, "Defender": 1, "Midfielder": 0, "Goalkeeper": 1}
  else:
    return {"Attacker": 0, "Defender": 0, "Midfielder": 0, "Goalkeeper": 1}


def get_robot_roles(blackboard, current_players, isDefence=False):
  """
    Takes in an array of player numbers and assigns roles to each player. If a player had a role already, it will
    try and take that role again. This helps with substitutions and potential player penalties.

    Returns:
      A dictionary containing the new roles of each player.
  """
  valid_players = []
  for player in current_players:
    if player != 0:
      valid_players.append(player)

  roles = blackboard.roles

  requirements = get_requirements(valid_players, isDefence)

  new_roles = {}
  new_players = []
  for player in valid_players:
    if player not in roles:
      new_players.append(player)
      continue

    if requirements[roles[player]] > 0:
      new_roles[player] = roles[player]
      requirements[roles[player]] -= 1
      continue

    # Player exists in roles but its role has been taken/removed.
    # Player tries to move to closest role possible in this case.

    preferences = []
    if roles[player] == "Attacker":
      preferences = ["Defender", "Goalkeeper"]
    elif roles[player] == "Midfielder":
      preferences = ["Defender", "Attacker", "Goalkeeper"]
    elif roles[player] == "Defender":
      preferences = ["Goalkeeper", "Attacker"]
    else:
      preferences = ["Defender", "Attacker"]

    for role in preferences:
        if requirements[role] > 0:
            new_roles[player] = role
            requirements[role] -= 1
            break

  # Iterate through new_players and assign to any roles that haven't been filled yet.

  for player in new_players:
    for role, count in requirements.items():
        if count > 0:
            new_roles[player] = role
            requirements[role] -= 1
            break

  return new_roles

