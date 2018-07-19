std_msgs/Header header

# Spline Start Time
# Defines the time when the spline should begin interpolation
# If set to Time(0), the spline should start imidiately when recieved
time start_time

# Time Vector
# Knot vectors should be the same length as the time vector
# Vector can be variably incrementing in time, but must maintain t(k) < t(k+1)
duration[] t

# Knots
float64[] x
float64[] y
float64[] z
float64[] yaw
