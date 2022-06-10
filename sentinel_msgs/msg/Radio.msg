# source of message
uint8 SENTINEL_ROBOT = 1
uint8 FOLLOWER_ROBOT = 2

# state of sender
uint8 NORMAL = 0
uint8 ANOMALY = 1
uint8 BASE = 2
uint8 SPEED_UP = 3
uint8 SLOW_DOWN = 4
uint8 STOP = 5
uint8 STARTING = 6
uint8 WAIT = 7

uint8 source
uint8 state
geometry_msgs/PoseStamped pose
float32 linear_speed 0.5
float32 angular_speed 0.5