# This is a named variant (struct) for CurrentState topic.
# Introduced by popular demand.

std_msgs/Header   header

# Translational states
float64[3]        pos_ned
float64[3]        vel_ned
float64[3]        acc_ned

# Rotational states
float64[3]        ang_rpy
float64[3]        rat_rpy
float64[3]        acc_rpy

# Origin offset from a base frame.
# This is where the local map origin was placed.
float64[3]        map_origin

# state validity
bool              state_valid