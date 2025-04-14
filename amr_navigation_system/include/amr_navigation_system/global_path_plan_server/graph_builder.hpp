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

#ifndef AMR_NAVIGATION_SYSTEM__GLOBAL_PATH_PLAN_SERVER__GRAPH_BUILDER_HPP_
#define AMR_NAVIGATION_SYSTEM__GLOBAL_PATH_PLAN_SERVER__GRAPH_BUILDER_HPP_

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <filesystem>  // For extracting the map name
#include <string>
#include <memory>

#include "amr_navigation_system/global_path_plan_server/amr_traffic_rules.hpp"  // Custom traffic rules for AMR
#include "amr_navigation_system/utils/common_utils.hpp"  // Include the GPSPoint struct for map origin
#include "amr_navigation_system/utils/logger.hpp"

namespace amr_navigation
{

/**
 * @brief The GraphBuilder class is responsible for loading an OSM map and building a routing graph
 *        based on the loaded map and custom traffic rules.
 */
class GraphBuilder
{
public:
  // Constructor and Destructor
  explicit GraphBuilder(std::shared_ptr<amr_logging::NodeLogger> logger);
  ~GraphBuilder() = default;

  /**
     * @brief Loads an OSM map from the specified file path and applies the given map origin for UTM projection.
     *
     * @param mapPath Path to the OSM map file.
     * @param mapOrigin The GPS origin (latitude, longitude) for UTM projection.
     * @return A shared pointer to the loaded Lanelet2 map.
     */
  lanelet::LaneletMapPtr loadOSMMap(const std::string & mapPath, const GPSPoint & mapOrigin);

  /**
     * @brief Builds a routing graph for the AMR based on the loaded map and custom traffic rules.
     *
     * @param map A pointer to the loaded Lanelet2 map.
     * @param amrTrafficRules Custom traffic rules for the AMR.
     * @param debugPath Path to the folder where debug files will be saved.
     * @param enableDebug If true, enables debug mode to output additional information.
     * @return A unique pointer to the built routing graph.
     */
  lanelet::routing::RoutingGraphUPtr buildGraph(
    const lanelet::LaneletMapPtr & map,
    const lanelet::traffic_rules::TrafficRules & amrTrafficRules, const std::string & debugPath,
    bool enableDebug = false);

private:
  /**
     * @brief Generates debug files for the routing graph, exporting it in various formats (GraphML, GraphViz, OSM).
     *
     * @param routingGraph The routing graph to debug.
     * @param debugFolderPath Path to the folder where debug files will be saved.
     * @return True if debugging information was successfully generated, false otherwise.
     */
  bool enableGraphDebug(
    const lanelet::routing::RoutingGraphUPtr & routingGraph, const std::string & debugFolderPath);

  // Store the origin for UTM projection during debugging (optional)
  double originLatitude_ = 0.0;
  double originLongitude_ = 0.0;
  std::string mapPath_;  // Path to the loaded map
  std::shared_ptr<amr_logging::NodeLogger> logger_;
};

}  // namespace amr_navigation

#endif  // AMR_NAVIGATION_SYSTEM__GLOBAL_PATH_PLAN_SERVER__GRAPH_BUILDER_HPP_
