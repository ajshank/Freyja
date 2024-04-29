# Custom msg format for logging state-action pairs suitable for
# typical RL pipelines. Experimental.
# -- aj / Mar 2024.

std_msgs/Header         header
geometry_msgs/Vector3   prev_state_pos
geometry_msgs/Vector3   prev_state_vel
geometry_msgs/Vector3   prev_action
geometry_msgs/Vector3   curr_state_pos
geometry_msgs/Vector3   curr_state_vel