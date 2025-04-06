#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <builtin_interfaces/msg/time.hpp>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace amr_logging
{

// Logger class for structured logging
class NodeLogger
{
public:
  // Constructor: Initializes the logger with a log file
  NodeLogger(
    const std::string & base_path, const std::string & package_name, const std::string & node_name,
    const std::string & file_type, int & log_level);

  // Destructor: Ensures proper shutdown of the logger
  ~NodeLogger();

  // Generic logging function
  void log(
    int level, const std::string & message, const std::string & file_name, uint32_t line_number,
    const std::string & function_name);

  // Convenience methods for each log level
  void log_debug(
    const std::string & message, const std::string & file_name, uint32_t line_number,
    const std::string & function_name);
  void log_info(
    const std::string & message, const std::string & file_name, uint32_t line_number,
    const std::string & function_name);
  void log_warn(
    const std::string & message, const std::string & file_name, uint32_t line_number,
    const std::string & function_name);
  void log_error(
    const std::string & message, const std::string & file_name, uint32_t line_number,
    const std::string & function_name);
  void log_fatal(
    const std::string & message, const std::string & file_name, uint32_t line_number,
    const std::string & function_name);

private:
  std::string log_file_path_;               // Path to the log file
  std::shared_ptr<spdlog::logger> logger_;  // Spdlog logger instance
  std::mutex log_mutex_;                    // Mutex for thread safety
  std::string package_name_;                // Name of the package
  std::string node_name_;                   // Name of the node
  std::string file_type_;
  int log_level_;  // Minimum log level to log

  // Helper functions
  std::string getLogLevelString(int & level);
  std::string getCurrentTimeString();

  // Flush logs periodically
  void flushLogs();
};

// Declare the expandTilde function
std::string expandTilde(const std::string & path);

}  // namespace amr_logging

#endif  // LOGGER_HPP
