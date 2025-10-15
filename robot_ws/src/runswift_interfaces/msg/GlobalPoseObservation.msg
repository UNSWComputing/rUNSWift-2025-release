# this message is sent to the SelfEstimator and contains an estimate for the
# robot's position and heading as given by some external reference (eg. center circle)

std_msgs/Header header
geometry_msgs/Point32 position
float32 heading

# todo: maybe add uncertainty hints?