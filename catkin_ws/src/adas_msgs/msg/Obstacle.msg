# Single obstacle detection result
# 单个障碍物检测结果

std_msgs/Header header

# Unique ID for tracking
uint32 id

# Classification: 0=Unknown, 1=Vehicle, 2=Pedestrian, 3=Cyclist, 4=Static
uint8 classification
uint8 UNKNOWN = 0
uint8 VEHICLE = 1
uint8 PEDESTRIAN = 2
uint8 CYCLIST = 3
uint8 STATIC = 4

# Position in ego vehicle frame (meters)
geometry_msgs/Point position

# Velocity in ego vehicle frame (m/s)
geometry_msgs/Vector3 velocity

# Bounding box dimensions (meters)
float32 width
float32 length
float32 height

# Distance to ego vehicle (meters)
float32 distance

# Confidence score [0.0, 1.0]
float32 confidence
