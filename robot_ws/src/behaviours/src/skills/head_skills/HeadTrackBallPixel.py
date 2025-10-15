import time

from src.skills.head_skills.SeHeadTrackBall import se_head_track_ball

def head_track_ball_pixel(blackboard):
    # Define parameters for the top camera
    TOP_PIXEL_CENTRE_X = 320
    TOP_PIXEL_CENTRE_Y = 240
    TOP_YAW_GAIN = 0.0002
    TOP_PITCH_GAIN = 0.0007
    TOP_BASELINE_PITCH = 0.25
    TOP_MIN_PITCH = 0.15
    TOP_MAX_PITCH = 0.45
    TOP_YAW_THRESHOLD = 100
    TOP_YAW_SCALE = 1.0
    TOP_YAW_PITCH_FACTOR = 0.02  # extra pitch at max yaw (radians)

    # Define parameters for the bottom camera
    BOTTOM_PIXEL_CENTRE_X = 160
    BOTTOM_PIXEL_CENTRE_Y = 120
    BOTTOM_YAW_GAIN = 0.0015
    BOTTOM_PITCH_GAIN = 0.0025
    BOTTOM_BASELINE_PITCH = 0.3
    BOTTOM_MIN_PITCH = 0.2
    BOTTOM_MAX_PITCH = 0.5
    BOTTOM_YAW_PITCH_FACTOR = 0.03  # extra pitch for bottom cam

    # Maximum speeds (radians per second)
    MAX_YAW_SPEED = 0.5
    MAX_PITCH_SPEED = 0.5

    vision_ball = blackboard.vision_ball
    if vision_ball is None or not vision_ball.ball_features:
        # blackboard.motion_command.head_command.is_relative = True
        # blackboard.motion_command.head_command.yaw = 0.0
        # blackboard.motion_command.head_command.yaw_speed = 0.0
        # blackboard.motion_command.head_command.pitch = TOP_BASELINE_PITCH
        # blackboard.motion_command.head_command.pitch_speed = 0.0
        # blackboard.motion_command.head_command.is_relative = False
        return False

    # pick camera params
    if vision_ball.header.frame_id == "base_footprint_from_top":
        centre_x, centre_y = TOP_PIXEL_CENTRE_X, TOP_PIXEL_CENTRE_Y
        base_yaw_gain, pitch_gain = TOP_YAW_GAIN, TOP_PITCH_GAIN
        baseline_pitch, min_pitch, max_pitch = TOP_BASELINE_PITCH, TOP_MIN_PITCH, TOP_MAX_PITCH
        yaw_pitch_factor = TOP_YAW_PITCH_FACTOR
    elif vision_ball.header.frame_id == "base_footprint_from_bottom":
        centre_x, centre_y = BOTTOM_PIXEL_CENTRE_X, BOTTOM_PIXEL_CENTRE_Y
        base_yaw_gain, pitch_gain = BOTTOM_YAW_GAIN, BOTTOM_PITCH_GAIN
        baseline_pitch, min_pitch, max_pitch = BOTTOM_BASELINE_PITCH, BOTTOM_MIN_PITCH, BOTTOM_MAX_PITCH
        yaw_pitch_factor = BOTTOM_YAW_PITCH_FACTOR
    else:
        blackboard.ctx.get_logger().error(
            f"Error: Invalid frame_id '{vision_ball.header.frame_id}' received. Doing nothing."
        )
        return False

    ball = vision_ball.ball_features[0]
    if time.time() - blackboard._last_vision_ball > 1.0:
        se_head_track_ball(blackboard)
        return
    x_off = ball.ball_pixel_coordinates.x - centre_x
    y_off = ball.ball_pixel_coordinates.y - centre_y

    # scale yaw gain on top camera
    if vision_ball.header.frame_id == "base_footprint_from_top" and abs(x_off) > TOP_YAW_THRESHOLD:
        effective_yaw_gain = base_yaw_gain * (
            1 + TOP_YAW_SCALE * ((abs(x_off) - TOP_YAW_THRESHOLD) / TOP_YAW_THRESHOLD)
        )
    else:
        effective_yaw_gain = base_yaw_gain

    # desired yaw & speed
    desired_yaw = -effective_yaw_gain * x_off
    yaw_speed = min(MAX_YAW_SPEED, abs(desired_yaw))

    # compute extra pitch offset based on yaw magnitude
    yaw_ratio = min(abs(desired_yaw) / MAX_YAW_SPEED, 1.0)
    extra_pitch = yaw_ratio * yaw_pitch_factor

    # desired pitch: baseline + vision pitch + yaw-induced bump
    desired_pitch = baseline_pitch + pitch_gain * y_off + extra_pitch
    desired_pitch = max(min_pitch, min(desired_pitch, max_pitch))
    pitch_speed = min(MAX_PITCH_SPEED, abs(desired_pitch - baseline_pitch))

    # send command
    cmd = blackboard.motion_command.head_command
    cmd.is_relative = True
    cmd.yaw = float(desired_yaw)
    cmd.yaw_speed = float(yaw_speed)
    cmd.pitch = float(desired_pitch)
    cmd.pitch_speed = float(pitch_speed)

    return True
