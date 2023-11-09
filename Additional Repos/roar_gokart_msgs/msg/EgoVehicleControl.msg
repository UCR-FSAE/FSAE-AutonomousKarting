# This represents a vehicle control message sent to ROAR Arduino

# 0. <= throttle <= 1.
float32 throttle

# -1. <= steer <= 1.
float32 steer

# 0. <= brake <= 1.
float32 brake

# reverse 0 or 1
bool reverse
