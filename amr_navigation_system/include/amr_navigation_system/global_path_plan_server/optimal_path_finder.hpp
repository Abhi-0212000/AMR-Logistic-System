#ifndef OPTIMAL_PATH_FINDER_HPP
#define OPTIMAL_PATH_FINDER_HPP

// Lanelet2 includes
#include <lanelet2_routing/RoutingGraph.h>  // For routing graph and pathfinding in Lanelet2
#include <lanelet2_core/LaneletMap.h>       // For accessing the map and its layers
#include <lanelet2_core/primitives/GPSPoint.h>  // For handling GPS coordinates
#include <lanelet2_projection/UTM.h>        // For GPS to UTM projection (conversion)
#include "amr_navigation_system/utils/common_utils.hpp"  // Include the GPSPoint struct

#include <boost/geometry.hpp>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Area.h>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>  // For ROS2 node-related functionalities and logging

namespace amr_navigation {

class OptimalPathFinder {
public:
    OptimalPathFinder();
    ~OptimalPathFinder();

    // Public method to get the optimal path between two lanelet or area IDs
    lanelet::routing::LaneletOrAreaPath getOptimalPath(
        const lanelet::LaneletMapPtr& map,
        const lanelet::routing::RoutingGraph& graph,
        const lanelet::Id& start_id,
        const lanelet::Id& end_id,
        bool start_is_lanelet,
        bool end_is_lanelet);

    // Method to build the path response including distance and time estimates
    PathResponse buildPathResponse(
        const lanelet::routing::LaneletOrAreaPath& path,
        const lanelet::traffic_rules::TrafficRules& amrTrafficRules);

    // Method to find the nearest lanelet or area given GPS coordinates
    NearestElement getNearestLaneletOrArea(
        const lanelet::LaneletMapPtr& map,
        const GPSPoint& gps_point,
        const GPSPoint& origin_point,
        double max_dist_threshold);

private:
    // Private method to convert GPS coordinates to UTM coordinates
    lanelet::BasicPoint2d convertGPSToUTM(const GPSPoint& gps_point, const GPSPoint& origin_point);

    // Private method to calculate the total distance of a path
    double calculatePathDistance(const lanelet::routing::LaneletOrAreaPath& path);

    // Private method to estimate the total travel time along a path, considering speed limits
    double estimateTravelTime(
        const lanelet::routing::LaneletOrAreaPath& path,
        const lanelet::traffic_rules::TrafficRules& amrTrafficRules);

    // Private helper method to calculate the length of a lanelet or area element
    double getLength(const lanelet::ConstLaneletOrArea& la) const;

    // Private helper method to estimate the travel time of a lanelet or area element
    double getTravelTime(
        const lanelet::ConstLaneletOrArea& la,
        const lanelet::traffic_rules::TrafficRules& amrTrafficRules) const;
};

}  // namespace amr_navigation

#endif  // OPTIMAL_PATH_FINDER_HPP
