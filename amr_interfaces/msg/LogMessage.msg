# LogMessage.msg
builtin_interfaces/Time stamp     # ROS Time stamp for synchronization
uint8 level                       # Log level (0: DEBUG, 1: INFO, 2: WARN, 3: ERROR, 4: FATAL)
string node_name                  # Name of the node sending the log
string package_name               # Name of the package
string message                    # The log message
string file_name                  # Source file name (optional)
uint32 line_number                # Line number in source file (optional)
string function_name              # Function name (optional)
string thread_id                  # Thread ID (optional)