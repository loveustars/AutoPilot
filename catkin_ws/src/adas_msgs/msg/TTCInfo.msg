# Time-to-Collision information
# 碰撞时间信息

std_msgs/Header header

# Time to collision (seconds), -1 if no collision risk
float32 ttc

# Distance to closest obstacle (meters)
float32 distance

# Relative velocity (m/s), positive = approaching
float32 relative_velocity

# Relative acceleration (m/s^2)
float32 relative_acceleration

# Is collision imminent
bool collision_imminent

# Confidence of TTC calculation [0.0, 1.0]
float32 confidence
