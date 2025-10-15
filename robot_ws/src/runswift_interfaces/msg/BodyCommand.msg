# Actions
# All the things behaviours will likely need
string DEFENDER_CENTRE   = "defender_centre"
string GOALIE_CENTRE     = "goalie_centre"
string GOALIE_DIVE_LEFT  = "goalie_dive_left"
string GOALIE_DIVE_RIGHT = "goalie_dive_right"
string GOALIE_FAST_SIT   = "goalie_fast_sit"
string GOALIE_INITIAL    = "goalie_initial"
string GOALIE_SIT        = "goalie_sit"
string GOALIE_STAND      = "goalie_stand"
string GOALIE_UNCENTRE   = "goalie_uncentre"
string KICK              = "kick"
string SIT               = "sit"
string STAND             = "stand"
string UNSTIFF           = "unstiff"
string WALK              = "walk"
string action_type

# Walks
geometry_msgs/Twist twist

bool bend                # for walks, bend = 1 with power !=0 crouches
bool use_shuffle         # takes steps that are low to the ground for obstacle avoidance
bool left_arm_behind
bool right_arm_behind
bool blocking            # tells the walk generator to take a big sidestep immediately
float32 speed            # caps the max speed where 0 = 66%, 1 = 100%

# Kicks
bool LEFT=0
bool RIGHT=1
bool foot
float64 power            # typically power for kick. for walk, sets standing stiffness

bool extra_stable_kick   # sacrifices execution time for stability by checking gyro

# More action types for motion debugging that behaviours *could* use
string NULL_GENERATOR    = "null_generator"
string GETUP_BACK        = "getup_back"
string GETUP_BACK_FAST   = "getup_back_fast"
string GETUP_BACK_SLOW   = "getup_back_slow"
string GETUP_FRONT       = "getup_front"
string GETUP_FRONT_FAST  = "getup_front_fast"
string GETUP_FRONT_SLOW  = "getup_front_slow"
string INITIAL           = "initial"
string MOTION_CALIBRATE  = "motion_calibrate"
string RAISE_ARM         = "raise_arm"
string REF_PICKUP        = "ref_pickup"
string TIP_OVER          = "tip_over"
string UKEMI_BACK        = "ukemi_back"
string UKEMI_FRONT       = "ukemi_front"
