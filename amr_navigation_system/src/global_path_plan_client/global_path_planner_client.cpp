#include "rclcpp/rclcpp.hpp"
#include "amr_interfaces/srv/compute_global_path.hpp"
#include <thread>

class GlobalPathPlannerClient : public rclcpp::Node {
public:
    GlobalPathPlannerClient() : Node("amr_global_path_planner_client") {
        // Declare parameters
        this->declare_parameter<double>("start_latitude", 50.71575968934);
        this->declare_parameter<double>("start_longitude", 10.46831175618);
        this->declare_parameter<double>("start_altitude", 0.0);
        this->declare_parameter<double>("end_latitude", 50.71506975439);
        this->declare_parameter<double>("end_longitude", 10.46775855057);
        this->declare_parameter<double>("end_altitude", 0.0);
        this->declare_parameter<bool>("use_time_based_routing", false);

        // Load parameters from YAML configuration file
        loadParameters();

        // Start the service call in a separate thread
        service_thread_ = std::thread(&GlobalPathPlannerClient::callGlobalPathPlannerService, this);
    }

    ~GlobalPathPlannerClient() {
        if (service_thread_.joinable()) {
            service_thread_.join();  // Ensure the thread finishes execution
        }
    }

    void loadParameters() {
        // Load the parameters
        this->get_parameter("start_latitude", start_latitude_);
        this->get_parameter("start_longitude", start_longitude_);
        this->get_parameter("start_altitude", start_altitude_);
        this->get_parameter("end_latitude", end_latitude_);
        this->get_parameter("end_longitude", end_longitude_);
        this->get_parameter("end_altitude", end_altitude_);
        this->get_parameter("use_time_based_routing", use_time_based_routing_);

        // Log loaded parameters
        RCLCPP_INFO(this->get_logger(), "Start Point (lat, lon, alt): (%.6f, %.6f, %.2f)", 
                    start_latitude_, start_longitude_, start_altitude_);
        RCLCPP_INFO(this->get_logger(), "End Point (lat, lon, alt): (%.6f, %.6f, %.2f)", 
                    end_latitude_, end_longitude_, end_altitude_);
    }
    
    void callGlobalPathPlannerService() {
        auto client = this->create_client<amr_interfaces::srv::ComputeGlobalPath>("global_path_planner");

        // Wait for the service to be available
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the service to be available...");
        }

        // Prepare the request
        auto request = std::make_shared<amr_interfaces::srv::ComputeGlobalPath::Request>();
        request->start_latitude = start_latitude_;
        request->start_longitude = start_longitude_;
        request->start_altitude = start_altitude_;
        request->end_latitude = end_latitude_;
        request->end_longitude = end_longitude_;
        request->end_altitude = end_altitude_;
        request->use_time_based_routing = use_time_based_routing_;

        // Send the request asynchronously
        auto future = client->async_send_request(request);

        // Get the response (blocking until response is received)
        try {
            auto response = future.get();  // Wait for the response
            if (response->status == 0) {
                RCLCPP_INFO(this->get_logger(), "Path planning successful");
                RCLCPP_INFO(this->get_logger(), "Total distance: %.2f meters", response->total_distance);
                RCLCPP_INFO(this->get_logger(), "Estimated time: %.2f seconds", response->estimated_time);

                // Print lanelet IDs and inversion status
                RCLCPP_INFO(this->get_logger(), "Lanelet IDs: ");
                for (const auto &id : response->lanelet_ids) {
                    RCLCPP_INFO(this->get_logger(), "%ld", id);  // Print each lanelet ID
                }

                RCLCPP_INFO(this->get_logger(), "Inverted statuses: ");
                for (size_t i = 0; i < response->is_inverted.size(); ++i) {
                    RCLCPP_INFO(this->get_logger(), "Lanelet ID %ld is inverted: %s", response->lanelet_ids[i], 
                                response->is_inverted[i] ? "true" : "false");
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Path planning failed. Status: %d, Message: %s", 
                            response->status, response->message.c_str());
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }

        // After receiving the response, shut down the node
        rclcpp::shutdown();
    }

private:
    std::thread service_thread_;  // Thread for the service call

    // Request parameters
    double start_latitude_;
    double start_longitude_;
    double start_altitude_;
    double end_latitude_;
    double end_longitude_;
    double end_altitude_;
    bool use_time_based_routing_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Load parameters from the YAML file
    auto options = rclcpp::NodeOptions().allow_undeclared_parameters(true)
                                        .automatically_declare_parameters_from_overrides(true);

    // Create the client node
    auto node = std::make_shared<GlobalPathPlannerClient>();

    // Spin the node (main thread)
    rclcpp::spin(node);

    // Ensure clean shutdown
    rclcpp::shutdown();
    return 0;
}
