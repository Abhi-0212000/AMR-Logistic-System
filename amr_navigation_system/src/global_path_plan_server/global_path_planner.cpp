#include "rclcpp/rclcpp.hpp"
#include "amr_interfaces/srv/compute_global_path.hpp"
#include "amr_navigation_system/global_path_plan_server/graph_builder.hpp"
#include "amr_navigation_system/utils/common_utils.hpp"  // Include the GPSPoint struct
#include "amr_navigation_system/global_path_plan_server/optimal_path_finder.hpp"

class GlobalPathPlanner : public rclcpp::Node 
{
public:
    GlobalPathPlanner() : Node("amr_global_path_planner") 
    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("paths.map_path", "/home/user/amr_data/maps");
        this->declare_parameter<std::string>("paths.log_path", "/home/user/amr_data/logs");
        this->declare_parameter<std::string>("paths.debug_path", "/home/user/amr_data/debug_data");
        this->declare_parameter<double>("origins.gps_origin.latitude", 0.0);
        this->declare_parameter<double>("origins.gps_origin.longitude", 0.0);
        this->declare_parameter<double>("origins.gps_origin.altitude", 0.0);
        this->declare_parameter<double>("bounding_box.min_lat", 0.0);
        this->declare_parameter<double>("bounding_box.min_lon", 0.0);
        this->declare_parameter<double>("bounding_box.max_lat", 0.0);
        this->declare_parameter<double>("bounding_box.max_lon", 0.0);
        this->declare_parameter<double>("max_distance_threshold", 50.0);

        // Load parameters from YAML
        loadParameters();

        // Create service
        server_ = this->create_service<amr_interfaces::srv::ComputeGlobalPath>(
            "global_path_planner",
            std::bind(&GlobalPathPlanner::callbackComputeGlobalPath, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), "Service server has been started.");
    }

private:
    void loadParameters()
    {
        // Load the parameters
        this->get_parameter("paths.map_path", map_path_);
        this->get_parameter("paths.log_path", log_path_);
        this->get_parameter("paths.debug_path", debug_path_);
        this->get_parameter("origins.gps_origin.latitude", origin_latitude_);
        this->get_parameter("origins.gps_origin.longitude", origin_longitude_);
        this->get_parameter("origins.gps_origin.altitude", origin_altitude_);
        this->get_parameter("bounding_box.min_lat", bbox_.min_lat);
        this->get_parameter("bounding_box.min_lon", bbox_.min_lon);
        this->get_parameter("bounding_box.max_lat", bbox_.max_lat);
        this->get_parameter("bounding_box.max_lon", bbox_.max_lon);
        this->get_parameter("max_distance_threshold", max_distance_threshold_);

        // Log loaded parameters
        RCLCPP_INFO(this->get_logger(), "Loaded Parameters: Map Path: %s, Origin (lat, lon, alt): (%.6f, %.6f, %.2f)", 
                    map_path_.c_str(), origin_latitude_, origin_longitude_, origin_altitude_);
        RCLCPP_INFO(this->get_logger(), "Bounding Box: (%.6f, %.6f) to (%.6f, %.6f)", 
                    bbox_.min_lat, bbox_.min_lon, bbox_.max_lat, bbox_.max_lon);
        RCLCPP_INFO(this->get_logger(), "Max Distance Threshold: %.2f", max_distance_threshold_);
    }

    void callbackComputeGlobalPath(const std::shared_ptr<amr_interfaces::srv::ComputeGlobalPath::Request> request,
                               std::shared_ptr<amr_interfaces::srv::ComputeGlobalPath::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Callback initiated.");

        try {
            amr_navigation::GPSPoint mapOrigin(origin_latitude_, origin_longitude_, origin_altitude_);
            amr_navigation::GPSPoint startGPSPoint(request->start_latitude, request->start_longitude); // Start GPS coordinates
            amr_navigation::GPSPoint endGPSPoint(request->end_latitude, request->end_longitude);   // End GPS coordinates

            // Validate latitude and longitude for UTM zone 32 (typically germany)
            // if (startGPSPoint.longitude < 6.0 || startGPSPoint.longitude > 12.0) {
            //     RCLCPP_ERROR(this->get_logger(), "Start point longitude out of valid UTM range.");
            //     response->status = 1;  // Indicate failure
            //     response->message = "Start point longitude out of valid UTM range.";
            //     return;
            // }
            // if (endGPSPoint.longitude < 6.0 || endGPSPoint.longitude > 12.0) {
            //     RCLCPP_ERROR(this->get_logger(), "End point longitude out of valid UTM range.");
            //     response->status = 1;  // Indicate failure
            //     response->message = "End point longitude out of valid UTM range.";
            //     return;
            // }

            // Check if start and end points are within bounds
            if (!amr_navigation::isWithinBounds(startGPSPoint, bbox_) || 
                !amr_navigation::isWithinBounds(endGPSPoint, bbox_)) {
                RCLCPP_ERROR(this->get_logger(), "Start or end point is outside the defined bounds.");
                response->status = 1;
                response->message = "Start or end point is outside the defined bounds.";
                return;
            }

            // Create an instance of GraphBuilder
            amr_navigation::GraphBuilder graph_builder;

            // Load the map using the parameters retrieved from the YAML file
            auto map = graph_builder.loadOSMMap(map_path_, mapOrigin);

            if (!map) {
                RCLCPP_ERROR(this->get_logger(), "Map loading failed.");
                response->status = 1;  // Indicate failure
                response->message = "Map loading failed.";
                return;
            }

            // Example of using a custom traffic rules instance
            auto amrTrafficRules = std::make_shared<lanelet::traffic_rules::AmrTrafficRules>();

            // Build the Graph from the loaded map
            auto routingGraph = graph_builder.buildGraph(map, *amrTrafficRules);

            // Create an instance of OptimalPathFinder
            amr_navigation::OptimalPathFinder optimal_path_finder;

            // Find nearest lanelet or area for start and end points
            amr_navigation::NearestElement startElement = optimal_path_finder.getNearestLaneletOrArea(map, startGPSPoint, mapOrigin, max_distance_threshold_);
            amr_navigation::NearestElement endElement = optimal_path_finder.getNearestLaneletOrArea(map, endGPSPoint, mapOrigin, max_distance_threshold_);

            // Compute the optimal path
            auto optimalPath = optimal_path_finder.getOptimalPath(map, *routingGraph, startElement.id, endElement.id,
                                                                    startElement.isLanelet, endElement.isLanelet);

            // Use a local variable to hold the response from buildPathResponse
            auto response_msg = optimal_path_finder.buildPathResponse(optimalPath, *amrTrafficRules);

            // Explicitly populate the response object for the client
            response->total_distance = response_msg.total_distance;
            response->estimated_time = response_msg.estimated_time;
            response->status = response_msg.status;
            response->message = response_msg.message;
            response->lanelet_ids = response_msg.lanelet_ids;
            response->is_inverted = response_msg.is_inverted;

        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Path planning failed: %s", e.what());
            response->status = 1;
            response->message = "Path planning failed: " + std::string(e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "An error occurred: %s", e.what());
            response->status = 1;
            response->message = "An error occurred: " + std::string(e.what());
        }
    }


    // Member variables to store parameter values
    std::string map_path_;
    std::string log_path_;
    std::string debug_path_;
    double origin_latitude_;
    double origin_longitude_;
    double origin_altitude_;
    amr_navigation::BoundingBox bbox_;
    double max_distance_threshold_;

    // Service definition
    rclcpp::Service<amr_interfaces::srv::ComputeGlobalPath>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Load the parameters from the YAML file
    auto options = rclcpp::NodeOptions().allow_undeclared_parameters(true)
                                        .automatically_declare_parameters_from_overrides(true)
                                        .parameter_overrides({
                                            {"paths.map_path", "/path/to/default/osm/file.osm"},
                                            {"origins.gps_origin.latitude", 0.0},
                                            {"origins.gps_origin.longitude", 0.0},
                                            {"origins.gps_origin.altitude", 0.0}
                                        });

    // Initialize the GlobalPathPlanner node with options
    auto node = std::make_shared<GlobalPathPlanner>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
