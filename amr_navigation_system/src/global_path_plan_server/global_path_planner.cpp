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

#include "amr_interfaces/srv/compute_global_path.hpp"
#include "amr_navigation_system/global_path_plan_server/graph_builder.hpp"
#include "amr_navigation_system/global_path_plan_server/optimal_path_finder.hpp"
#include "amr_navigation_system/utils/common_utils.hpp"  // Include the GPSPoint struct
#include "amr_navigation_system/utils/logger.hpp"
#include "rclcpp/rclcpp.hpp"

// This class defines the Global Path Planner Node
class GlobalPathPlanner : public rclcpp::Node
{
public:
  // Constructor to initialize the node and service server
  GlobalPathPlanner()
  : Node("amr_global_path_planner")
  {
    // Declare parameters with default values, loaded from the YAML file
    this->declare_parameter<std::string>("paths.map_path", "");
    this->declare_parameter<std::string>("paths.debug_path", amr_navigation::getDefaultDebugPath());
    this->declare_parameter<std::string>("log_data.log_path", "~/.local/share/AMR/logs");
    this->declare_parameter<std::string>("log_data.log_file_type", "json");
    this->declare_parameter<int>("log_data.log_level", 1);
    this->declare_parameter<bool>("debug_options.export_built_graph", false);
    this->declare_parameter<double>("origins.gps_origin.latitude", 0.0);
    this->declare_parameter<double>("origins.gps_origin.longitude", 0.0);
    this->declare_parameter<double>("origins.gps_origin.altitude", 0.0);
    this->declare_parameter<double>("bounding_box.min_lat", 0.0);
    this->declare_parameter<double>("bounding_box.min_lon", 0.0);
    this->declare_parameter<double>("bounding_box.max_lat", 0.0);
    this->declare_parameter<double>("bounding_box.max_lon", 0.0);
    this->declare_parameter<double>("max_distance_threshold", 50.0);

    // Load the declared parameters
    loadParameters();

    map_path_ = amr_navigation::expandTilde(map_path_);

    logger_ = std::make_shared<amr_logging::NodeLogger>(
      log_path_, "amr_navigation_system", "amr_global_path_planner", log_file_type_, log_level_);

    // Check map file and create debug directory if needed
    checkMapAndCreateDebugDir();

    // Create the service that provides global path planning
    server_ = this->create_service<amr_interfaces::srv::ComputeGlobalPath>(
      "global_path_planner", std::bind(
        &GlobalPathPlanner::callbackComputeGlobalPath, this,
        std::placeholders::_1, std::placeholders::_2));

    // Log information using both RCLCPP and the new logger
    RCLCPP_INFO(this->get_logger(), "Service server has been started.");
    logger_->log_info(
      "Service server 'global_path_planner' has been started.", __FILE__, __LINE__, __FUNCTION__);
  }

private:
  // Function to load parameters from YAML file or override them
  void loadParameters()
  {
    this->get_parameter("paths.map_path", map_path_);
    this->get_parameter_or("paths.debug_path", debug_path_, amr_navigation::getDefaultDebugPath());
    this->get_parameter("log_data.log_path", log_path_);
    this->get_parameter("log_data.log_file_type", log_file_type_);
    this->get_parameter("log_data.log_level", log_level_);
    this->get_parameter("debug_options.export_built_graph", export_built_graph_);
    this->get_parameter("origins.gps_origin.latitude", origin_latitude_);
    this->get_parameter("origins.gps_origin.longitude", origin_longitude_);
    this->get_parameter("origins.gps_origin.altitude", origin_altitude_);
    this->get_parameter("bounding_box.min_lat", bbox_.min_lat);
    this->get_parameter("bounding_box.min_lon", bbox_.min_lon);
    this->get_parameter("bounding_box.max_lat", bbox_.max_lat);
    this->get_parameter("bounding_box.max_lon", bbox_.max_lon);
    this->get_parameter("max_distance_threshold", max_distance_threshold_);
  }

  void checkMapAndCreateDebugDir()
  {
    if (!amr_navigation::checkFileExists(map_path_)) {
      logger_->log_error("Map file does not exist: " + map_path_, __FILE__, __LINE__, __FUNCTION__);
      throw std::runtime_error("Map file not found");
    }

    if (export_built_graph_) {
      try {
        logger_->log_info("Using debug path: " + debug_path_, __FILE__, __LINE__, __FUNCTION__);
        amr_navigation::createDirectory(debug_path_);
        logger_->log_info(
          "Debug directory created: " + debug_path_, __FILE__, __LINE__, __FUNCTION__);
      } catch (const std::exception & e) {
        logger_->log_error(
          "Failed to create debug directory: " + debug_path_ + ". Error: " + e.what(), __FILE__,
          __LINE__, __FUNCTION__);
        throw;
      }
    }
  }

  void callbackComputeGlobalPath(
    const std::shared_ptr<amr_interfaces::srv::ComputeGlobalPath::Request> request,
    std::shared_ptr<amr_interfaces::srv::ComputeGlobalPath::Response> response)
  {
    logger_->log_info("Callback initiated.", __FILE__, __LINE__, __FUNCTION__);
    RCLCPP_INFO(this->get_logger(), "Received a Client Request. Finding Optimal Path ...");

    try {
      amr_navigation::GPSPoint mapOrigin(origin_latitude_, origin_longitude_, origin_altitude_);
      amr_navigation::GPSPoint startGPSPoint(request->start_latitude, request->start_longitude);
      amr_navigation::GPSPoint endGPSPoint(request->end_latitude, request->end_longitude);

      if (
        !amr_navigation::isWithinBounds(startGPSPoint, bbox_) ||
        !amr_navigation::isWithinBounds(endGPSPoint, bbox_))
      {
        logger_->log_error(
          "Start or end point is outside the defined bounds.", __FILE__, __LINE__, __FUNCTION__);
        response->status = 1;
        response->message = "Start or end point is outside the defined bounds.";
        return;
      }

      amr_navigation::GraphBuilder graph_builder(logger_);
      auto map = graph_builder.loadOSMMap(map_path_, mapOrigin);

      // If map loading fails, handle the error
      if (!map) {
        logger_->log_error("Failed to load the map data.", __FILE__, __LINE__, __FUNCTION__);
        response->status = 1;
        response->message = "Failed to load the map data.";
        return;
      }

      // Initialize custom traffic rules for AMR
      auto amrTrafficRules = std::make_shared<lanelet::traffic_rules::AmrTrafficRules>();

      // Build the routing graph from the map
      auto routingGraph =
        graph_builder.buildGraph(map, *amrTrafficRules, debug_path_, export_built_graph_);

      // Use OptimalPathFinder to find the nearest lanelet or area for start and end
      amr_navigation::OptimalPathFinder optimal_path_finder(logger_);
      auto startElement = optimal_path_finder.getNearestLaneletOrArea(
        map, startGPSPoint, mapOrigin, max_distance_threshold_);
      auto endElement = optimal_path_finder.getNearestLaneletOrArea(
        map, endGPSPoint, mapOrigin, max_distance_threshold_);

      // Compute the optimal path between start and end
      auto optimalPath = optimal_path_finder.getOptimalPath(
        map, *routingGraph, startElement.id, endElement.id, startElement.isLanelet,
        endElement.isLanelet);

      // Build the path response to send back to the client
      auto response_msg = optimal_path_finder.buildPathResponse(optimalPath, *amrTrafficRules);

      // Set the response for the client
      response->total_distance = response_msg.total_distance;
      response->estimated_time = response_msg.estimated_time;
      response->status = response_msg.status;
      response->message = response_msg.message;
      response->lanelet_ids = response_msg.lanelet_ids;
      response->is_inverted = response_msg.is_inverted;

      logger_->log_info("Path computed successfully.", __FILE__, __LINE__, __FUNCTION__);
      RCLCPP_INFO(
        this->get_logger(), "Path Comupted Successfully and sent the response back to client");
    } catch (const std::runtime_error & e) {
      // Handle runtime errors during path planning
      logger_->log_error(
        "Error during path planning: " + std::string(e.what()), __FILE__, __LINE__, __FUNCTION__);
      response->status = 1;
      response->message = "Path planning failed: " + std::string(e.what());
    } catch (const std::exception & e) {
      // Handle generic exceptions
      logger_->log_error(
        "Error during path planning: " + std::string(e.what()), __FILE__, __LINE__, __FUNCTION__);
      response->status = 1;
      response->message = "An error occurred: " + std::string(e.what());
    }
  }

  std::shared_ptr<amr_logging::NodeLogger> logger_;
  rclcpp::Service<amr_interfaces::srv::ComputeGlobalPath>::SharedPtr server_;

  // Parameters for loading the map
  std::string map_path_;
  double origin_latitude_;
  double origin_longitude_;
  double origin_altitude_;

  // for exporting the graph
  std::string debug_path_;
  bool export_built_graph_;
  // for logging
  std::string log_path_;
  std::string log_file_type_;
  int log_level_;

  amr_navigation::BoundingBox bbox_;
  double max_distance_threshold_;
};

// Main function to initialize the node and spin
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Load parameters and initialize the node
  auto options = rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)
    .parameter_overrides(
    {{"paths.map_path", "/path/to/default/osm/file.osm"},
      {"origins.gps_origin.latitude", 0.0},
      {"origins.gps_origin.longitude", 0.0},
      {"origins.gps_origin.altitude", 0.0}});

  // Create a shared pointer for the GlobalPathPlanner node
  auto node = std::make_shared<GlobalPathPlanner>();

  rclcpp::on_shutdown(
    []() {RCLCPP_INFO(rclcpp::get_logger("main"), "Node is shutting down...");});

  // Spin the node to handle incoming requests
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
