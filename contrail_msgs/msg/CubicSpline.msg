std_msgs/Header header

# Spline Start Time
# Defines the time when the spline should begin interpolation
# If set to Time(0), the spline should start imidiately when recieved
std_msgs/Time start_time

# Time Vector
# Knot vectors should be the same length as the time vector
std_msgs/Duration[] t

# Knots
float64[] x
float64[] y
float64[] z
float64[] yaw
