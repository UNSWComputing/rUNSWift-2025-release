uint8 NONE = 0
uint8 LINE = 1
uint8 CORNER = 2
uint8 T_JUNCTION = 3
uint8 PENALTY_SPOT = 4
uint8 CENTRE_CIRCLE = 5
uint8 FIELD_LINE_POINT = 6
uint8 X_JUNCTION = 7
uint8 PARALLEL_LINE = 8
uint8 GOAL_BOX_CORNER = 9
uint8 PENALTY_BOX_CORNER = 10

uint8 type
uint8 confidence_score
geometry_msgs/Pose feature_coordinates

vision_msgs/Point2D feature_pixel_coordinates
