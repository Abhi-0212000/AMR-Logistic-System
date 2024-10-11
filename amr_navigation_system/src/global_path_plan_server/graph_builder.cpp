#include "amr_navigation_system/global_path_plan_server/graph_builder.hpp"

namespace amr_navigation {

lanelet::LaneletMapPtr GraphBuilder::loadOSMMap(const std::string& mapPath, const GPSPoint& mapOrigin) {
    // Store origin coordinates for use during debug
    originLatitude_ = mapOrigin.latitude;
    originLongitude_ = mapOrigin.longitude;
    mapPath_ = mapPath;

    // Load the map using the UTM projector with the specified origin
    try {
        lanelet::LaneletMapPtr map_ptr = lanelet::load(
            mapPath,
            lanelet::projection::UtmProjector(lanelet::Origin({originLatitude_, originLongitude_}))
        );
        RCLCPP_INFO(rclcpp::get_logger("GraphBuilder"), "Map loaded successfully from %s.", mapPath.c_str());
        return map_ptr;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("GraphBuilder"), "Failed to load OSM map from %s: %s", mapPath.c_str(), e.what());
        return nullptr;
    }
}

lanelet::routing::RoutingGraphUPtr GraphBuilder::buildGraph(const lanelet::LaneletMapPtr& map, const lanelet::traffic_rules::TrafficRules& amrTrafficRules, bool enableDebug) {
    // Check if the map is valid
    if (!map) {
        RCLCPP_ERROR(rclcpp::get_logger("GraphBuilder"), "Cannot build graph: Map is null.");
        return nullptr;
    }

    // Create routing graph based on the map and traffic rules
    auto routingGraph = lanelet::routing::RoutingGraph::build(*map, amrTrafficRules);
    RCLCPP_INFO(rclcpp::get_logger("GraphBuilder"), "Routing graph built successfully.");

    // If debug is enabled, call the enableGraphDebug method
    if (enableDebug) {
        if (enableGraphDebug(routingGraph, "path_to_debug_folder")) {  // Replace with actual debug folder path
            RCLCPP_INFO(rclcpp::get_logger("GraphBuilder"), "Debugging information generated successfully.");
        } else {
            RCLCPP_WARN(rclcpp::get_logger("GraphBuilder"), "Failed to generate debugging information.");
        }
    }

    return routingGraph;
}

bool GraphBuilder::enableGraphDebug(const lanelet::routing::RoutingGraphUPtr& routingGraph, const std::string& debugFolderPath) {
    // Ensure routingGraph is valid
    if (!routingGraph) {
        RCLCPP_WARN(rclcpp::get_logger("GraphBuilder"), "Routing graph is null, debug information cannot be generated.");
        return false;
    }

    // Extract the map name from mapPath
    std::string mapName = std::filesystem::path(mapPath_).stem().string();  // Get the file name without extension

    // Use the map name to create unique filenames for debug files
    std::filesystem::path graphMLPath = std::filesystem::path(debugFolderPath) / (mapName + "_routing_graph.graphml");
    std::filesystem::path graphVizPath = std::filesystem::path(debugFolderPath) / (mapName + "_routing_graph.gv");
    std::filesystem::path osmPath = std::filesystem::path(debugFolderPath) / (mapName + "_routing_graph.osm");

    try {
        // Export to GraphML
        routingGraph->exportGraphML(graphMLPath);
        RCLCPP_INFO(rclcpp::get_logger("GraphBuilder"), "Exported graph to GraphML: %s", graphMLPath.string().c_str());

        // Export to GraphViz
        routingGraph->exportGraphViz(graphVizPath);
        RCLCPP_INFO(rclcpp::get_logger("GraphBuilder"), "Exported graph to GraphViz: %s", graphVizPath.string().c_str());

        // Export to OSM using the stored origin coordinates
        lanelet::LaneletMapConstPtr debugLaneletMap = routingGraph->getDebugLaneletMap();
        lanelet::projection::UtmProjector projector(lanelet::Origin({originLatitude_, originLongitude_}));  // Use stored origin values
        lanelet::write(osmPath, *debugLaneletMap, projector);
        RCLCPP_INFO(rclcpp::get_logger("GraphBuilder"), "Exported debug lanelet map to OSM: %s", osmPath.string().c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("GraphBuilder"), "Error while exporting debug information: %s", e.what());
        return false;
    }

    return true;
}

} // namespace amr_navigation
