# This message also can represent a fixed-distance (binary) ranger.  This
# sensor will have min_range===max_range===distance of detection.
# These sensors follow REP 117 and will output -Inf if the object is detected
# and +Inf if the object is outside of the detection range.

float32 master_speed	# speed of the master
float32 accel_x			# acceleration x
float32 accel_y			# acceleration y
float32 accel_z			# acceleration z
