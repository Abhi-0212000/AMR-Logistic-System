#include "amr_navigation_system/global_path_plan_server/graph_builder.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_navigation {


GraphBuilder::GraphBuilder(std::shared_ptr<amr_logging::NodeLogger> logger)
    : logger_(logger) {}

lanelet::LaneletMapPtr GraphBuilder::loadOSMMap(const std::string& mapPath, const GPSPoint& mapOrigin) {
    logger_->log_info("Loading OSM Map...", __FILE__, __LINE__, __FUNCTION__);
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
        logger_->log_info(fmt::format("Map loaded successfully from {}.", mapPath), __FILE__, __LINE__, __FUNCTION__);
        return map_ptr;
    } catch (const std::exception& e) {
        // Log error if map loading fails
        logger_->log_error(fmt::format("Failed to load OSM map from {}: {}", mapPath, e.what()), __FILE__, __LINE__, __FUNCTION__);
        return nullptr;
    }
}


lanelet::routing::RoutingGraphUPtr GraphBuilder::buildGraph(
    const lanelet::LaneletMapPtr& map, 
    const lanelet::traffic_rules::TrafficRules& amrTrafficRules,
    const std::string& debugPath,
    bool enableDebug
) {
    // Check if the map is valid before proceeding
    if (!map) {
        logger_->log_error("Cannot build graph: Map is null.", __FILE__, __LINE__, __FUNCTION__);
        return nullptr;
    }

    try {
        // Create routing graph using Lanelet2 and AMR-specific traffic rules
        auto routingGraph = lanelet::routing::RoutingGraph::build(*map, amrTrafficRules);
        logger_->log_info("Routing graph built successfully.", __FILE__, __LINE__, __FUNCTION__);

        // If debug mode is enabled, generate debugging information
        if (enableDebug) {
            if (enableGraphDebug(routingGraph, debugPath)) {
                logger_->log_info("Debugging information generated successfully.", __FILE__, __LINE__, __FUNCTION__);
            } else {
                logger_->log_warn("Failed to generate debugging information.", __FILE__, __LINE__, __FUNCTION__);
            }
        }

        return routingGraph;
    } catch (const std::exception& e) {
        // Log error if routing graph fails to build
        logger_->log_error(fmt::format("Failed to build routing graph: {}.", e.what()), __FILE__, __LINE__, __FUNCTION__);
        return nullptr;
    }
}


bool GraphBuilder::enableGraphDebug(const lanelet::routing::RoutingGraphUPtr& routingGraph, const std::string& debugFolderPath) {
    // Ensure routingGraph is valid
    if (!routingGraph) {
        logger_->log_warn("Routing graph is null, debug information cannot be generated.", __FILE__, __LINE__, __FUNCTION__);
        return false;
    }

    // Extract the map name from mapPath to name the debug files
    std::string mapName = std::filesystem::path(mapPath_).stem().string();  // Get the file name without extension

    // Use the map name to create unique filenames for debug files
    std::filesystem::path graphMLPath = std::filesystem::path(debugFolderPath) / (mapName + "_routing_graph.graphml");
    std::filesystem::path graphVizPath = std::filesystem::path(debugFolderPath) / (mapName + "_routing_graph.gv");
    std::filesystem::path osmPath = std::filesystem::path(debugFolderPath) / (mapName + "_routing_graph.osm");

    try {
        // Export to GraphML format
        routingGraph->exportGraphML(graphMLPath);
        logger_->log_info(fmt::format("Exported graph to GraphML: {}.", graphMLPath.string().c_str()), __FILE__, __LINE__, __FUNCTION__);

        // Export to GraphViz format
        routingGraph->exportGraphViz(graphVizPath);
        logger_->log_info(fmt::format("Exported graph to GraphViz: {}.", graphVizPath.string().c_str()), __FILE__, __LINE__, __FUNCTION__);

        // Export to OSM format using the stored UTM origin
        lanelet::LaneletMapConstPtr debugLaneletMap = routingGraph->getDebugLaneletMap();
        lanelet::projection::UtmProjector projector(lanelet::Origin({originLatitude_, originLongitude_}));
        lanelet::write(osmPath, *debugLaneletMap, projector);
        logger_->log_info(fmt::format("Exported debug lanelet map to OSM: {}.", osmPath.string().c_str()), __FILE__, __LINE__, __FUNCTION__);
    } catch (const std::exception& e) {
        // Handle any errors that occur during exporting
        logger_->log_error(fmt::format("Error while exporting debug information: {}.", e.what()), __FILE__, __LINE__, __FUNCTION__);
        return false;
    }

    return true;
}

}  // namespace amr_navigation
