std_msgs/Header header

# Spline Start Time
# Defines the time when the spline should begin interpolation
# If set to Time(0), the spline should start imidiately when recieved
time start_time

# Duration of the spline
duration duration

# Knots
# Each knot is placed evenly throughout the duration of the spline
float64[] x
float64[] y
float64[] z
float64[] yaw
