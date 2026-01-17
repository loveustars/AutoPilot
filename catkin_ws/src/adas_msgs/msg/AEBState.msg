# AEB System State
# AEB系统状态

std_msgs/Header header

# Current FSM state
uint8 state
uint8 INACTIVE = 0       # TTC > 2.5s
uint8 WARNING = 1        # 1.5s <= TTC < 2.5s (FCW)
uint8 PARTIAL_BRAKE = 2  # 0.6s <= TTC < 1.5s
uint8 FULL_BRAKE = 3     # TTC < 0.6s

# Commanded deceleration (m/s^2)
float32 deceleration_cmd

# State duration (seconds)
float32 state_duration

# TTC at state entry
float32 ttc_at_entry

# Is system enabled
bool enabled

# Fault status
bool fault_detected
string fault_message
