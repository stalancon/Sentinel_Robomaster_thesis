
uint8 NORMAL = 0
uint8 ANOMALY = 1
uint8 BASE = 2
uint8 SPEED_UP = 3
uint8 SLOW_DOWN = 4
uint8 STOP = 5
uint8 STARTING = 6
uint8 WAIT = 7

uint8 state
nav_msgs/Path path
float32 linear_speed 0.5      # 0.5 m/s
float32 angular_speed 0.5  #  30 deg/s
