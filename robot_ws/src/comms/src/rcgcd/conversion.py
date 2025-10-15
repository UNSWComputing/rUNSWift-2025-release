from rcgcd.robocup_game_control_data import MAX_NUM_PLAYERS, RoboCupGameControlData
from runswift_interfaces.msg import CommsRCGCD


def rcgcd_data_to_msg(data: bytes) -> CommsRCGCD:
    """Convert binary data to RCGCRD ROS msg."""
    parsed = RoboCupGameControlData.parse(data)
    msg = CommsRCGCD()
    msg.packet_number = parsed.packetNumber
    msg.players_per_team = parsed.playersPerTeam
    msg.competition_phase = parsed.competitionPhase
    msg.competition_type = parsed.competitionType
    msg.game_phase = parsed.gamePhase
    msg.state = parsed.state
    msg.set_play = parsed.setPlay
    msg.first_half = parsed.firstHalf
    msg.kicking_team = parsed.kickingTeam
    msg.secs_remaining = parsed.secsRemaining
    msg.secondary_time = parsed.secondaryTime
    for t in range(2):
        msg.teams[t].team_number = parsed.teams[t].teamNumber
        msg.teams[t].field_player_colour = parsed.teams[t].fieldPlayerColour
        msg.teams[t].goalkeeper_colour = parsed.teams[t].goalkeeperColour
        msg.teams[t].goalkeeper = parsed.teams[t].goalkeeper
        msg.teams[t].score = parsed.teams[t].score
        msg.teams[t].penalty_shot = parsed.teams[t].penaltyShot
        msg.teams[t].single_shots = parsed.teams[t].singleShots
        msg.teams[t].message_budget = parsed.teams[t].messageBudget
        for p in range(MAX_NUM_PLAYERS):
            msg.teams[t].players[p].penalty = parsed.teams[t].players[p].penalty
            msg.teams[t].players[p].secs_till_unpenalised = parsed.teams[t].players[p].secsTillUnpenalised
    return msg

