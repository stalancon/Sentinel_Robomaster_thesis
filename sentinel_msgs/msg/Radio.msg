uint8 SENTINEL_ROBOT = 1
uint8 FOLLOWER_ROBOT = 2

uint8 FINE = 0
uint8 ANOMALY = 1
uint8 SLOW_DOWN = 3

uint8 source
uint8 state
geometry_msgs/PoseStamped pose
float32 linear_speed 0.5
float32 angular_speed 0.5
