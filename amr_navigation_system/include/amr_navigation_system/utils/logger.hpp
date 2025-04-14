// Copyright 2025 Abhishek Nannuri
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AMR_NAVIGATION_SYSTEM__UTILS__LOGGER_HPP_
#define AMR_NAVIGATION_SYSTEM__UTILS__LOGGER_HPP_

#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <fstream>
#include <memory>
#include <mutex>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

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

#endif  // AMR_NAVIGATION_SYSTEM__UTILS__LOGGER_HPP_
