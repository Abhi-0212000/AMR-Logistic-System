#include "amr_navigation_system/utils/logger.hpp"
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <thread>
#include <spdlog/async.h>
#include <pwd.h> // Include this header for getpwuid
#include "rclcpp/rclcpp.hpp"
#include <cstring> // Add this line for strerror and strlen
#include <cerrno>  // Add this line for errno
#include <cstdlib>

namespace amr_logging {

// bool operator<(LogLevel lhs, int rhs) {
//     return static_cast<int>(lhs) < rhs;
// }

std::string expandTilde(const std::string& path) {
    if (!path.empty() && path[0] == '~') {
        const char* home = getenv("HOME");
        if (!home) {
            home = getpwuid(getuid())->pw_dir;
        }
        return std::string(home) + path.substr(1);
    }
    return path;
}

NodeLogger::NodeLogger(const std::string& base_path, const std::string& package_name, 
               const std::string& node_name, const std::string& file_type, 
               int& log_level)
    : package_name_(package_name), node_name_(node_name), file_type_(file_type), log_level_(log_level) {
    
    // Expand tilde in base path
    std::string expanded_base_path = expandTilde(base_path);
    
    // Create log directory structure
    std::string log_dir = expanded_base_path + "/" + package_name + "/" + node_name;
    // std::filesystem::create_directories(log_dir);
    if (std::filesystem::exists(log_dir)) {
    std::cout << "Log directory already exists: " << log_dir << std::endl;
    } else {
        // Try to create the directories
        if (std::filesystem::create_directories(log_dir)) {
            std::cout << "Directories created successfully: " << log_dir << std::endl;
        } else {
            std::cerr << "Failed to create directories: " << strerror(errno) << " " << log_dir << std::endl;
        }
    }
    
    // Construct log file path with timestamp
    log_file_path_ = log_dir + "/log_" + getCurrentTimeString() + "." + file_type_;
    std::cout << log_file_path_ << std::endl;

    // Initialize the spdlog thread pool
    size_t thread_pool_size = 1; // Adjust the size as needed
    size_t queue_size = 32; // Adjust the size as needed
    spdlog::init_thread_pool(queue_size, thread_pool_size);

    // Create an asynchronous logger with a file sink
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_file_path_, true);
    logger_ = std::make_shared<spdlog::async_logger>("async_logger", file_sink, spdlog::thread_pool(), spdlog::async_overflow_policy::block);
    spdlog::register_logger(logger_);
    logger_->set_pattern("%v");  // Set log message pattern

    // Set the spdlog log level based on log_level_**
    switch (log_level_) {
        case 0:  // DEBUG
            logger_->set_level(spdlog::level::debug);
            break;
        case 1:  // INFO
            logger_->set_level(spdlog::level::info);
            break;
        case 2:  // WARN
            logger_->set_level(spdlog::level::warn);
            break;
        case 3:  // ERROR
            logger_->set_level(spdlog::level::err);
            break;
        case 4:  // FATAL
            logger_->set_level(spdlog::level::critical);
            break;
        default:  // Default to INFO if an invalid log level is passed
            logger_->set_level(spdlog::level::info);
            break;
    }

}

NodeLogger::~NodeLogger() {
    if (logger_) {
        logger_->flush();  // Flush any remaining log messages
        RCLCPP_INFO(rclcpp::get_logger("NodeLogger"), "Log Messages are flushed to the log file. shutting down the spdlog.");
        spdlog::shutdown();  // Gracefully shutdown the logger and join background threads
    }
}

void NodeLogger::log(int level, const std::string& message, const std::string& file_name, 
                  uint32_t line_number, const std::string& function_name) {
    
    // Return if the log level is below the minimum set level
    if (level < log_level_) {
        return;
    }

    // Lock the mutex to ensure thread safety
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    std::string log_entry;

    if (file_type_ == "json") {
        nlohmann::json json_log;
        json_log["timestamp"] = getCurrentTimeString();
        json_log["level"] = getLogLevelString(level);
        json_log["node_name"] = node_name_;
        json_log["package_name"] = package_name_;
        json_log["message"] = message;
        json_log["file_name"] = file_name;
        json_log["line_number"] = line_number;
        json_log["function_name"] = function_name;
        json_log["thread_id"] = std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id()));
        log_entry = json_log.dump(4); // JSON format. Pretty print JSON with 4 spaces indentation
    } else {
        log_entry = getCurrentTimeString() + " [" + getLogLevelString(level) + "] " +
                    node_name_ + " " + package_name_ + ": " + message +
                    " (" + file_name + ":" + std::to_string(line_number) + 
                    " in " + function_name + ")";
    }
    switch(level) {
        case 0:  // DEBUG
            logger_->debug(log_entry);
            break;
        case 1:  // INFO
            logger_->info(log_entry);
            break;
        case 2:  // WARN
            logger_->warn(log_entry);
            break;
        case 3:  // ERROR
            logger_->error(log_entry);
            break;
        case 4:  // FATAL
            logger_->critical(log_entry);
            break;
    }
}

void NodeLogger::log_debug(const std::string& message, const std::string& file_name, 
                       uint32_t line_number, const std::string& function_name) {
    log(0, message, file_name, line_number, function_name);  // DEBUG level = 0
}

void NodeLogger::log_info(const std::string& message, const std::string& file_name, 
                      uint32_t line_number, const std::string& function_name) {
    log(1, message, file_name, line_number, function_name);  // INFO level = 1
}

void NodeLogger::log_warn(const std::string& message, const std::string& file_name, 
                      uint32_t line_number, const std::string& function_name) {
    log(2, message, file_name, line_number, function_name);  // WARN level = 2
}

void NodeLogger::log_error(const std::string& message, const std::string& file_name, 
                       uint32_t line_number, const std::string& function_name) {
    log(3, message, file_name, line_number, function_name);  // ERROR level = 3
}

void NodeLogger::log_fatal(const std::string& message, const std::string& file_name, 
                       uint32_t line_number, const std::string& function_name) {
    log(4, message, file_name, line_number, function_name);  // FATAL level = 4
}

std::string NodeLogger::getLogLevelString(int& level) {
    switch (level) {
        case 0: return "DEBUG";
        case 1: return "INFO";
        case 2: return "WARN";
        case 3: return "ERROR";
        case 4: return "FATAL";
        default: return "UNKNOWN";
    }
}

std::string NodeLogger::getCurrentTimeString() {
    // Get current time with milliseconds precision
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    // Format time as YYYY-MM-DD_HH-MM-SS.milliseconds
    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&now_time_t), "%Y-%m-%d_%H-%M-%S") // Use gmtime for UTC
        << '.' << std::setfill('0') << std::setw(3) << now_ms.count()
        << " UTC"; // Append time zone
    return oss.str();
}

} // namespace amr_logging
