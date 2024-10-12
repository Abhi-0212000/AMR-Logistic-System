#include "rclcpp/rclcpp.hpp"
#include "amr_interfaces/srv/compute_global_path.hpp"
#include "amr_navigation_system/global_path_plan_server/graph_builder.hpp"
#include "amr_navigation_system/utils/common_utils.hpp"  // Include the GPSPoint struct
#include "amr_navigation_system/global_path_plan_server/optimal_path_finder.hpp"

// This class defines the Global Path Planner Node
class GlobalPathPlanner : public rclcpp::Node 
{
public:
    // Constructor to initialize the node and service server
    GlobalPathPlanner() : Node("amr_global_path_planner") 
    {
        // Declare parameters with default values, loaded from the YAML file
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

        // Load the declared parameters
        loadParameters();

        // Create the service that provides global path planning
        server_ = this->create_service<amr_interfaces::srv::ComputeGlobalPath>(
            "global_path_planner",
            std::bind(&GlobalPathPlanner::callbackComputeGlobalPath, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), "Service server has been started.");
    }

private:
    // Function to load parameters from YAML file or override them
    void loadParameters()
    {
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

        // Logging the loaded parameters for visibility
        RCLCPP_INFO(this->get_logger(), "Loaded Parameters: Map Path: %s, Origin (lat, lon, alt): (%.6f, %.6f, %.2f)", 
                    map_path_.c_str(), origin_latitude_, origin_longitude_, origin_altitude_);
        RCLCPP_INFO(this->get_logger(), "Bounding Box: (%.6f, %.6f) to (%.6f, %.6f)", 
                    bbox_.min_lat, bbox_.min_lon, bbox_.max_lat, bbox_.max_lon);
        RCLCPP_INFO(this->get_logger(), "Max Distance Threshold: %.2f", max_distance_threshold_);
    }

    // Callback function for the service request
    void callbackComputeGlobalPath(
        const std::shared_ptr<amr_interfaces::srv::ComputeGlobalPath::Request> request,
        std::shared_ptr<amr_interfaces::srv::ComputeGlobalPath::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Callback initiated.");

        try {
            // Creating GPS points for start and end from the request
            amr_navigation::GPSPoint mapOrigin(origin_latitude_, origin_longitude_, origin_altitude_);
            amr_navigation::GPSPoint startGPSPoint(request->start_latitude, request->start_longitude); 
            amr_navigation::GPSPoint endGPSPoint(request->end_latitude, request->end_longitude);   

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

            // Initialize GraphBuilder to load the map
            amr_navigation::GraphBuilder graph_builder;
            auto map = graph_builder.loadOSMMap(map_path_, mapOrigin);

            // If map loading fails, handle the error
            if (!map) {
                RCLCPP_ERROR(this->get_logger(), "Map loading failed.");
                response->status = 1;
                response->message = "Map loading failed.";
                return;
            }

            // Initialize custom traffic rules for AMR
            auto amrTrafficRules = std::make_shared<lanelet::traffic_rules::AmrTrafficRules>();

            // Build the routing graph from the map
            auto routingGraph = graph_builder.buildGraph(map, *amrTrafficRules);

            // Use OptimalPathFinder to find the nearest lanelet or area for start and end
            amr_navigation::OptimalPathFinder optimal_path_finder;
            auto startElement = optimal_path_finder.getNearestLaneletOrArea(map, startGPSPoint, mapOrigin, max_distance_threshold_);
            auto endElement = optimal_path_finder.getNearestLaneletOrArea(map, endGPSPoint, mapOrigin, max_distance_threshold_);

            // Compute the optimal path between start and end
            auto optimalPath = optimal_path_finder.getOptimalPath(map, *routingGraph, startElement.id, endElement.id, startElement.isLanelet, endElement.isLanelet);

            // Build the path response to send back to the client
            auto response_msg = optimal_path_finder.buildPathResponse(optimalPath, *amrTrafficRules);


            // Set the response for the client
            response->total_distance = response_msg.total_distance;
            response->estimated_time = response_msg.estimated_time;
            response->status = response_msg.status;
            response->message = response_msg.message;
            response->lanelet_ids = response_msg.lanelet_ids;
            response->is_inverted = response_msg.is_inverted;

        } catch (const std::runtime_error& e) {
            // Handle runtime errors during path planning
            RCLCPP_ERROR(this->get_logger(), "Path planning failed: %s", e.what());
            response->status = 1;
            response->message = "Path planning failed: " + std::string(e.what());
        } catch (const std::exception& e) {
            // Handle generic exceptions
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

// Main function to initialize the node and spin
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Load parameters and initialize the node
    auto options = rclcpp::NodeOptions().allow_undeclared_parameters(true)
                                        .automatically_declare_parameters_from_overrides(true)
                                        .parameter_overrides({
                                            {"paths.map_path", "/path/to/default/osm/file.osm"},
                                            {"origins.gps_origin.latitude", 0.0},
                                            {"origins.gps_origin.longitude", 0.0},
                                            {"origins.gps_origin.altitude", 0.0}
                                        });

    // Create a shared pointer for the GlobalPathPlanner node
    auto node = std::make_shared<GlobalPathPlanner>();

    // Spin the node to handle incoming requests
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}