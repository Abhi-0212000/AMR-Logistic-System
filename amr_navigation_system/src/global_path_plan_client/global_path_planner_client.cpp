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

#include <thread>

#include "amr_interfaces/srv/compute_global_path.hpp"
#include "amr_navigation_system/utils/logger.hpp"
#include "rclcpp/rclcpp.hpp"

class GlobalPathPlannerClient : public rclcpp::Node
{
public:
  GlobalPathPlannerClient()
  : Node("amr_global_path_planner_client")
  {
    initializeParameters();
    loadParameters();
    initializeLogger();
    startServiceThread();
  }

  ~GlobalPathPlannerClient() {cleanup();}

private:
  // Member variables
  std::thread service_thread_;
  std::shared_ptr<amr_logging::NodeLogger> logger_;

  // GPS coordinates
  struct GpsPoint
  {
    double latitude;
    double longitude;
    double altitude;
  };
  GpsPoint start_point_;
  GpsPoint end_point_;

  // Configuration
  bool use_time_based_routing_;
  std::string log_path_;
  std::string log_file_type_;
  int log_level_;

  // Initialize parameter declarations
  void initializeParameters()
  {
    // GPS parameters
    this->declare_parameter<double>("start_latitude", 50.71575968934);
    this->declare_parameter<double>("start_longitude", 10.46831175618);
    this->declare_parameter<double>("start_altitude", 0.0);
    this->declare_parameter<double>("end_latitude", 50.71506975439);
    this->declare_parameter<double>("end_longitude", 10.46775855057);
    this->declare_parameter<double>("end_altitude", 0.0);

    // Routing parameters
    this->declare_parameter<bool>("use_time_based_routing", false);

    // Logger parameters
    this->declare_parameter<std::string>("log_data.log_path", "~/.local/share/AMR/logs");
    this->declare_parameter<std::string>("log_data.log_file_type", "json");
    this->declare_parameter<int>("log_data.log_level", 1);
  }

  // Load parameters from ROS
  void loadParameters()
  {
    // Load GPS coordinates
    this->get_parameter("start_latitude", start_point_.latitude);
    this->get_parameter("start_longitude", start_point_.longitude);
    this->get_parameter("start_altitude", start_point_.altitude);
    this->get_parameter("end_latitude", end_point_.latitude);
    this->get_parameter("end_longitude", end_point_.longitude);
    this->get_parameter("end_altitude", end_point_.altitude);

    // Load routing configuration
    this->get_parameter("use_time_based_routing", use_time_based_routing_);

    // Load logger configuration
    this->get_parameter("log_data.log_path", log_path_);
    this->get_parameter("log_data.log_file_type", log_file_type_);
    this->get_parameter("log_data.log_level", log_level_);
  }

  // Initialize logger
  void initializeLogger()
  {
    logger_ = std::make_shared<amr_logging::NodeLogger>(
      log_path_, "amr_navigation_system", "global_path_planner_client", log_file_type_, log_level_);

    // Log initial parameters
    logGpsCoordinates();
    RCLCPP_INFO(
      this->get_logger(), "Client node to the server 'global_path_planner' has been started.");
    logger_->log_info(
      "Client node to the server 'global_path_planner' has been started.", __FILE__, __LINE__,
      __FUNCTION__);
  }

  void logGpsCoordinates()
  {
    logger_->log_debug(
      fmt::format(
        "Start Point (lat, lon, alt): ({:.6f}, {:.6f}, {:.6f})", start_point_.latitude,
        start_point_.longitude, start_point_.altitude),
      __FILE__, __LINE__, __FUNCTION__);
    logger_->log_debug(
      fmt::format(
        "End Point (lat, lon, alt): ({:.6f}, {:.6f}, {:.6f})", end_point_.latitude,
        end_point_.longitude, end_point_.altitude),
      __FILE__, __LINE__, __FUNCTION__);
  }

  // Start service thread
  void startServiceThread()
  {
    service_thread_ = std::thread(&GlobalPathPlannerClient::callGlobalPathPlannerService, this);
  }

  // Clean up resources
  void cleanup()
  {
    if (service_thread_.joinable()) {
      service_thread_.join();
    }
    if (logger_) {
      RCLCPP_INFO(this->get_logger(), "Shutting down the client node and Flushing the logs...");
      logger_->log_info(
        "Shutting down the client node and Flushing the logs...", __FILE__, __LINE__, __FUNCTION__);
    }
  }

  // Create service request
  auto createServiceRequest()
  {
    auto request = std::make_shared<amr_interfaces::srv::ComputeGlobalPath::Request>();
    request->start_latitude = start_point_.latitude;
    request->start_longitude = start_point_.longitude;
    request->start_altitude = start_point_.altitude;
    request->end_latitude = end_point_.latitude;
    request->end_longitude = end_point_.longitude;
    request->end_altitude = end_point_.altitude;
    request->use_time_based_routing = use_time_based_routing_;
    return request;
  }

  // Handle successful response
  void handleSuccessfulResponse(
    const amr_interfaces::srv::ComputeGlobalPath::Response::SharedPtr & response)
  {
    logger_->log_info("Path planning successful", __FILE__, __LINE__, __FUNCTION__);
    logger_->log_info(
      fmt::format("Total distance: {:.2f} meters", response->total_distance), __FILE__, __LINE__,
      __FUNCTION__);
    logger_->log_info(
      fmt::format("Estimated time: {:.2f} seconds", response->estimated_time), __FILE__, __LINE__,
      __FUNCTION__);

    // Log lanelet IDs
    std::string lanelet_ids_str =
      fmt::format("Lanelet IDs: [{}]", fmt::join(response->lanelet_ids, ", "));
    logger_->log_info(lanelet_ids_str, __FILE__, __LINE__, __FUNCTION__);

    // Log inversion statuses
    std::vector<std::string> inversion_statuses;
    for (size_t i = 0; i < response->is_inverted.size(); ++i) {
      inversion_statuses.push_back(
        fmt::format(
          "{}: {}", response->lanelet_ids[i], response->is_inverted[i] ? "true" : "false"));
    }
    std::string inversion_statuses_str =
      fmt::format("Inverted statuses: [{}]", fmt::join(inversion_statuses, ", "));
    logger_->log_info(inversion_statuses_str, __FILE__, __LINE__, __FUNCTION__);
  }

  // Handle failed response
  void handleFailedResponse(
    const amr_interfaces::srv::ComputeGlobalPath::Response::SharedPtr & response)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Path planning failed. Status: %d, Message: %s", response->status,
      response->message.c_str());
    logger_->log_error(
      fmt::format(
        "Path planning failed. Status: {}, Message: {}", response->status,
        response->message.c_str()),
      __FILE__, __LINE__, __FUNCTION__);
  }

  // Main service call function
  void callGlobalPathPlannerService()
  {
    auto client =
      this->create_client<amr_interfaces::srv::ComputeGlobalPath>("global_path_planner");

    // Wait for service availability
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        logger_->log_error(
          "Interrupted while waiting for the service. Exiting.", __FILE__, __LINE__, __FUNCTION__);
        return;
      }
      RCLCPP_WARN(this->get_logger(), "Waiting for the service to be available...");
      logger_->log_warn(
        "Waiting for the service to be available...", __FILE__, __LINE__, __FUNCTION__);
    }

    auto request = createServiceRequest();
    auto future = client->async_send_request(request);

    try {
      auto response = future.get();
      if (response->status == 0) {
        handleSuccessfulResponse(response);
      } else {
        handleFailedResponse(response);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
      logger_->log_error(
        fmt::format("Service call failed: {}", e.what()), __FILE__, __LINE__, __FUNCTION__);
    }

    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<GlobalPathPlannerClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
