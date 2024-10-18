#ifndef OPTIMAL_PATH_FINDER_HPP
#define OPTIMAL_PATH_FINDER_HPP

// Lanelet2 includes
#include <lanelet2_routing/RoutingGraph.h>  // For routing graph and pathfinding in Lanelet2
#include <lanelet2_core/LaneletMap.h>       // For accessing the map and its layers
#include <lanelet2_core/primitives/GPSPoint.h>  // For handling GPS coordinates
#include <lanelet2_projection/UTM.h>        // For GPS to UTM projection (conversion)
#include "amr_navigation_system/utils/common_utils.hpp"  // Include custom GPSPoint struct

#include <boost/geometry.hpp>
#include <lanelet2_core/geometry/Lanelet.h>  // Geometry handling for lanelets
#include <lanelet2_core/geometry/Area.h>     // Geometry handling for areas

// ROS2 includes
#include <rclcpp/rclcpp.hpp>  // For ROS2 node functionalities and logging
#include "amr_navigation_system/utils/logger.hpp"

namespace amr_navigation {

// Class responsible for finding optimal paths in Lanelet2 maps
class OptimalPathFinder {
public:
    // Constructor and Destructor
    OptimalPathFinder(std::shared_ptr<amr_logging::NodeLogger> logger);
    ~OptimalPathFinder();

    /**
     * @brief Get the optimal path between two lanelet or area IDs
     * 
     * @param map Shared pointer to the Lanelet2 map
     * @param graph Lanelet2 routing graph
     * @param start_id Starting lanelet/area ID
     * @param end_id Ending lanelet/area ID
     * @param start_is_lanelet Flag indicating if the start element is a lanelet
     * @param end_is_lanelet Flag indicating if the end element is a lanelet
     * @return Optimal path between start and end as a LaneletOrAreaPath
     */
    lanelet::routing::LaneletOrAreaPath getOptimalPath(
        const lanelet::LaneletMapPtr& map,
        const lanelet::routing::RoutingGraph& graph,
        const lanelet::Id& start_id,
        const lanelet::Id& end_id,
        bool start_is_lanelet,
        bool end_is_lanelet);

    /**
     * @brief Build the path response including distance and time estimates
     * 
     * @param path The computed LaneletOrArea path
     * @param amrTrafficRules Traffic rules applicable to the path (e.g., speed limits)
     * @return PathResponse containing distance, time, and status information
     */
    PathResponse buildPathResponse(
        const lanelet::routing::LaneletOrAreaPath& path,
        const lanelet::traffic_rules::TrafficRules& amrTrafficRules);

    /**
     * @brief Find the nearest lanelet or area to a given GPS point
     * 
     * @param map Shared pointer to the Lanelet2 map
     * @param gps_point GPS coordinates for the search
     * @param origin_point GPS point for UTM conversion reference
     * @param max_dist_threshold Maximum allowable distance for a valid nearby element
     * @return Nearest lanelet or area found within the threshold
     */
    NearestElement getNearestLaneletOrArea(
        const lanelet::LaneletMapPtr& map,
        const GPSPoint& gps_point,
        const GPSPoint& origin_point,
        double max_dist_threshold);

private:
    // Converts GPS coordinates to UTM coordinates based on a reference origin
    lanelet::BasicPoint2d convertGPSToUTM(const GPSPoint& gps_point, const GPSPoint& origin_point);

    // Calculates the total distance of a LaneletOrArea path
    double calculatePathDistance(const lanelet::routing::LaneletOrAreaPath& path);

    // Estimates the total travel time of a path based on traffic rules (speed limits)
    double estimateTravelTime(
        const lanelet::routing::LaneletOrAreaPath& path,
        const lanelet::traffic_rules::TrafficRules& amrTrafficRules);

    // Helper method to calculate the length of a lanelet or area element
    double getLength(const lanelet::ConstLaneletOrArea& la) const;

    // Helper method to estimate the travel time of a lanelet or area element
    double getTravelTime(
        const lanelet::ConstLaneletOrArea& la,
        const lanelet::traffic_rules::TrafficRules& amrTrafficRules) const;
    
    std::shared_ptr<amr_logging::NodeLogger> logger_;
};

}  // namespace amr_navigation

#endif  // OPTIMAL_PATH_FINDER_HPP
