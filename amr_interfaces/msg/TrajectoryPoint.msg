# Used to publish individual trajectory points at high frequency (0.01s) to the MPC controller.

# The Header provides essential metadata about the message, which includes: 
# timestamp: When the message was created.
# frame_id: The reference frame for the message (e.g., map, base_link)

std_msgs/Header header
geometry_msgs/PoseStamped pose          # Single future pose (x, y, z, orientation)
float64 velocity                        # Desired velocity at this pose
float64 yaw_rate                        # Desired yaw rate at this pose
float64 timestamp                       # Time at which to reach this pose


# =========================== MORE DETAILED VERSION OF SAME INTERFACE DESCRIBED ABOVE ===========================
# std_msgs/Header header                                                                                        #   
#         builtin_interfaces/Time stamp                                                                         #
#                 int32 sec                                                                                     #
#                 uint32 nanosec                                                                                #
#         string frame_id                                                                                       #
# geometry_msgs/PoseStamped pose          # Single future pose (x, y, z, orientation)                           #
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
# float64 velocity                        # Desired velocity at this pose                                       #
# float64 yaw_rate                        # Desired yaw rate at this pose                                       #
# float64 timestamp                       # Time at which to reach this pose                                    #
# =============================================================================================================== 