# Safety Monitor Status
# 安全监控状态

std_msgs/Header header

# Overall system health
bool system_healthy

# Heartbeat status
bool primary_node_alive
float32 last_heartbeat_age  # seconds since last heartbeat

# Plausibility check results
bool command_plausible
string plausibility_reason

# Override active (safety taking control)
bool override_active

# Safe state command if override
float32 safe_deceleration

# Watchdog timeout threshold (ms)
float32 watchdog_timeout_ms
