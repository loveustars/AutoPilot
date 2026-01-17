# Array of detected obstacles
# 障碍物数组

std_msgs/Header header

# All detected obstacles
adas_msgs/Obstacle[] obstacles

# Closest obstacle index (-1 if none)
int32 closest_obstacle_idx
