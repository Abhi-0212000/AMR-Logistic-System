#ifndef GRAPH_BUILDER_HPP
#define GRAPH_BUILDER_HPP

#include "rclcpp/rclcpp.hpp"
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include "amr_navigation_system/global_path_plan_server/amr_traffic_rules.hpp" // custom traffic rules
#include <string>
#include <filesystem>  // For extracting the map name
#include "amr_navigation_system/utils/common_utils.hpp"  // Include the GPSPoint struct

namespace amr_navigation {

class GraphBuilder {
public:
    // Constructor and Destructor
    GraphBuilder() = default;
    ~GraphBuilder() = default;

    // Function to load the .osm map
    lanelet::LaneletMapPtr loadOSMMap(const std::string& mapPath, const GPSPoint& mapOrigin);

    // Function to build the routing graph
    lanelet::routing::RoutingGraphUPtr buildGraph(const lanelet::LaneletMapPtr& map, const lanelet::traffic_rules::TrafficRules& amrTrafficRules, bool enableDebug=false);

private:
    // Optional: Function to enable debug mode
    bool enableGraphDebug(const lanelet::routing::RoutingGraphUPtr& routingGraph, const std::string& debugFolderPath);

    // Store the origin for UTM projection during debugging (optional)
    double originLatitude_ = 0.0;
    double originLongitude_ = 0.0;
    std::string mapPath_;
};

} // namespace amr_navigation

#endif // GRAPH_BUILDER_HPP
