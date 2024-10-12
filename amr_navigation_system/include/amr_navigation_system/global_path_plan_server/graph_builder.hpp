#ifndef GRAPH_BUILDER_HPP
#define GRAPH_BUILDER_HPP

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include "amr_navigation_system/global_path_plan_server/amr_traffic_rules.hpp"  // Custom traffic rules for AMR
#include <string>
#include <filesystem>  // For extracting the map name
#include "amr_navigation_system/utils/common_utils.hpp"  // Include the GPSPoint struct for map origin

namespace amr_navigation {

/**
 * @brief The GraphBuilder class is responsible for loading an OSM map and building a routing graph 
 *        based on the loaded map and custom traffic rules.
 */
class GraphBuilder {
public:
    // Constructor and Destructor
    GraphBuilder() = default;
    ~GraphBuilder() = default;

    /**
     * @brief Loads an OSM map from the specified file path and applies the given map origin for UTM projection.
     * 
     * @param mapPath Path to the OSM map file.
     * @param mapOrigin The GPS origin (latitude, longitude) for UTM projection.
     * @return A shared pointer to the loaded Lanelet2 map.
     */
    lanelet::LaneletMapPtr loadOSMMap(const std::string& mapPath, const GPSPoint& mapOrigin);

    /**
     * @brief Builds a routing graph for the AMR based on the loaded map and custom traffic rules.
     * 
     * @param map A pointer to the loaded Lanelet2 map.
     * @param amrTrafficRules Custom traffic rules for the AMR.
     * @param enableDebug If true, enables debug mode to output additional information.
     * @return A unique pointer to the built routing graph.
     */
    lanelet::routing::RoutingGraphUPtr buildGraph(const lanelet::LaneletMapPtr& map, const lanelet::traffic_rules::TrafficRules& amrTrafficRules, bool enableDebug = false);

private:
    /**
     * @brief Generates debug files for the routing graph, exporting it in various formats (GraphML, GraphViz, OSM).
     * 
     * @param routingGraph The routing graph to debug.
     * @param debugFolderPath Path to the folder where debug files will be saved.
     * @return True if debugging information was successfully generated, false otherwise.
     */
    bool enableGraphDebug(const lanelet::routing::RoutingGraphUPtr& routingGraph, const std::string& debugFolderPath);

    // Store the origin for UTM projection during debugging (optional)
    double originLatitude_ = 0.0;
    double originLongitude_ = 0.0;
    std::string mapPath_;  // Path to the loaded map
};

}  // namespace amr_navigation

#endif  // GRAPH_BUILDER_HPP
