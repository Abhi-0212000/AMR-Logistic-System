# Used to communicate the robot’s current position, orientation, and velocity 
# from the localization package to Other packages (amr_local_planner)


# The Header provides essential metadata about the message, which includes: 
# timestamp: When the message was created.
# frame_id: The reference frame for the message (e.g., map, base_link)


std_msgs/Header header
geometry_msgs/PoseStamped pose          # Position and orientation of the robot
float64 velocity                        # Current velocity of the robot. Linear velocity in the x direction.
float64 yaw_rate                        # Current yaw rate. Angular velocity around the z-axis.

# YAW_RATE ===> the angular velocity around the vertical axis of the vehicle. 
#               it measures how quickly the vehicle is rotating or turning.
#               It is measured in radians per second (rad/s) or degrees per second (°/s).

# =========================== MORE DETAILED VERSION OF SAME INTERFACE DESCRIBED ABOVE ===========================
# std_msgs/Header header                                                                                        #
#         builtin_interfaces/Time stamp                                                                         #
#                 int32 sec                                                                                     #
#                 uint32 nanosec                                                                                #
#         string frame_id                                                                                       #
# geometry_msgs/PoseStamped pose          # Position and orientation of the robot                               #
#         std_msgs/Header header                                                                                #
#                 builtin_interfaces/Time stamp                                                                 #
#                         int32 sec                                                                             #
#                         uint32 nanosec                                                                        #
#                 string frame_id                                                                               #
#         Pose pose                                                                                             #
#                 Point position                                                                                #
#                         float64 x                                                                             #
#                         float64 y                                                                             #
#                         float64 z                                                                             #
#                 Quaternion orientation                                                                        #
#                         float64 x 0                                                                           #
#                         float64 y 0                                                                           #
#                         float64 z 0                                                                           #
#                         float64 w 1                                                                           #
# float64 velocity                        # Current velocity of the robot                                       #
# float64 yaw_rate                        # Current yaw rate                                                    #
# =============================================================================================================== 