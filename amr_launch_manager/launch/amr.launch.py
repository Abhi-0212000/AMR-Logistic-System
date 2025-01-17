from calendar import c
from threading import local
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal)"
    )
    
    # Get package directories using FindPackageShare
    # dummy_loc_config = PathJoinSubstitution([
    #     FindPackageShare("amr_local_planner"),
    #     "config",
    #     "dummy_localization_params.yaml"
    # ])
    dummy_loc_config = "/home/abhi/ros2_lanelet2_ws/src/AMR-Logistic-System/amr_local_planner/config/dummy_localization_params.yaml"
    
    # global_planner_config = PathJoinSubstitution([
    #     FindPackageShare("amr_navigation_system"),
    #     "config",
    #     "global_path_planner_params.yaml"
    # ])
    global_planner_config = "/home/abhi/ros2_lanelet2_ws/src/AMR-Logistic-System/amr_navigation_system/config/global_path_planner_params.yaml"
    
    # local_planner_config = PathJoinSubstitution([
    #     FindPackageShare("amr_local_planner"),
    #     "config",
    #     "local_planner_params.yaml"
    # ])
    local_planner_config = "/home/abhi/ros2_lanelet2_ws/src/AMR-Logistic-System/amr_local_planner/config/local_planner_params.yaml"
    
    # central_mgmt_config = PathJoinSubstitution([
    #     FindPackageShare("amr_central_management"),
    #     "config",
    #     "central_management_params.yaml"
    # ])
    central_mgmt_config = "/home/abhi/ros2_lanelet2_ws/src/AMR-Logistic-System/amr_central_management/config/central_management_params.yaml"

    # Define nodes
    dummy_localization_node = Node(
        package="amr_local_planner",
        executable="amr_dummy_localization",
        name="amr_dummy_localization",
        output="screen",
        parameters=[dummy_loc_config],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        respawn=False,  # Don't respawn on failure
    )

    global_path_planner_node = Node(
        package="amr_navigation_system",
        executable="global_path_planner",
        name="amr_global_path_planner",
        output="screen",
        parameters=[global_planner_config],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        respawn=False,
    )

    local_planner_node = Node(
        package="amr_local_planner",
        executable="amr_local_path_planner",
        name="amr_local_path_planner",
        output="screen",
        parameters=[local_planner_config],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        respawn=False,
    )

    central_management_node = Node(
        package="amr_central_management",
        executable="central_management_node",
        name="central_management_node",
        output="screen",
        parameters=[central_mgmt_config],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        respawn=False,
    )

    # Event handlers for sequential launch and error handling and Cascading Shutdown Effect.
    # Event Handlers provide continuous monitoring during runtime.
    event_handlers = [
        # Handler for dummy_localization_node startup
        RegisterEventHandler(
            OnProcessStart(
                target_action=dummy_localization_node,
                on_start=[
                    LogInfo(msg="✓ dummy_localization_node started successfully"),
                    LogInfo(msg="Launching global_path_planner_node..."),
                    global_path_planner_node,
                ],
            )
        ),
        
        # Handler for dummy_localization_node failure
        RegisterEventHandler(
            OnProcessExit(
                target_action=dummy_localization_node,
                on_exit=[
                    LogInfo(msg="✗ dummy_localization_node failed to start or crashed"),
                    Shutdown(
                        reason="dummy_localization_node failed. Check logs and configuration."
                    ),
                ]
            )
        ),

        # Handler for global_path_planner_node startup
        RegisterEventHandler(
            OnProcessStart(
                target_action=global_path_planner_node,
                on_start=[
                    LogInfo(msg="✓ global_path_planner_node started successfully"),
                    LogInfo(msg="Launching local_planner_node..."),
                    local_planner_node,
                ],
            )
        ),
        
        # Handler for global_path_planner_node failure
        RegisterEventHandler(
            OnProcessExit(
                target_action=global_path_planner_node,
                on_exit=[
                    LogInfo(msg="✗ global_path_planner_node failed to start or crashed"),
                    Shutdown(
                        reason="global_path_planner_node failed. Check logs and configuration."
                    ),
                ]
            )
        ),

        # Handler for local_planner_node startup
        RegisterEventHandler(
            OnProcessStart(
                target_action=local_planner_node,
                on_start=[
                    LogInfo(msg="✓ local_planner_node started successfully"),
                    LogInfo(msg="Launching central_management_node..."),
                    central_management_node,
                ],
            )
        ),
        
        # Handler for local_planner_node failure
        RegisterEventHandler(
            OnProcessExit(
                target_action=local_planner_node,
                on_exit=[
                    LogInfo(msg="✗ local_planner_node failed to start or crashed"),
                    Shutdown(
                        reason="local_planner_node failed. Check logs and configuration."
                    ),
                ]
            )
        ),

        # Handler for central_management_node startup
        RegisterEventHandler(
            OnProcessStart(
                target_action=central_management_node,
                on_start=[
                    LogInfo(msg="✓ All nodes started successfully!"),
                ],
            )
        ),
        
        # Handler for central_management_node failure
        RegisterEventHandler(
            OnProcessExit(
                target_action=central_management_node,
                on_exit=[
                    LogInfo(msg="✗ central_management_node failed to start or crashed"),
                    Shutdown(
                        reason="central_management_node failed. Check logs and configuration."
                    ),
                ]
            )
        ),
    ]

    return LaunchDescription([log_level_arg, dummy_localization_node] + event_handlers)