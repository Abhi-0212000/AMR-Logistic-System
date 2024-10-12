#include "amr_navigation_system/global_path_plan_server/optimal_path_finder.hpp"
#include <limits>
#include <stdexcept> // For runtime_error

namespace amr_navigation {

// Constructor
OptimalPathFinder::OptimalPathFinder() {}

// Destructor
OptimalPathFinder::~OptimalPathFinder() {}

lanelet::routing::LaneletOrAreaPath OptimalPathFinder::getOptimalPath(
    const lanelet::LaneletMapPtr& map,
    const lanelet::routing::RoutingGraph& graph,
    const lanelet::Id& start_id,
    const lanelet::Id& end_id,
    bool start_is_lanelet,
    bool end_is_lanelet) {

    lanelet::Optional<lanelet::routing::LaneletOrAreaPath> bestPath;
    double shortestDistance = std::numeric_limits<double>::max();

    // Lambda function to try different path combinations (inverted start/end)
    auto tryPath = [&](const lanelet::ConstLaneletOrArea& start, const lanelet::ConstLaneletOrArea& end) {
        auto path = graph.shortestPathIncludingAreas(start, end, 0, false);
        if (path) {
            // Log path IDs for debugging purposes
            std::stringstream path_ids;
            for (const auto& element : path.get()) {
                path_ids << element.id() << " -> ";
            }
            path_ids << "END";
            
            RCLCPP_INFO(rclcpp::get_logger("OptimalPathFinder"), 
                        "Trying path with IDs: %s", path_ids.str().c_str());

            // Calculate and log the path distance
            double distance = calculatePathDistance(path.get());
            RCLCPP_INFO(rclcpp::get_logger("OptimalPathFinder"), 
                        "Path distance: %.2f meters.", distance);

            if (distance < shortestDistance) {
                bestPath = path;
                shortestDistance = distance;
                RCLCPP_INFO(rclcpp::get_logger("OptimalPathFinder"), 
                            "New shortest path found with distance: %.2f meters.", shortestDistance);
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("OptimalPathFinder"), 
                        "No valid path found from start ID %ld to end ID %ld.", start.id(), end.id());
        }
    };

    RCLCPP_INFO(rclcpp::get_logger("OptimalPathFinder"), 
                "Finding optimal path from start ID %ld to end ID %ld.", start_id, end_id);

    // Try different combinations of lanelet and area inversion for start and end
    if (start_is_lanelet) {
        lanelet::ConstLanelet start = map->laneletLayer.get(start_id);
        if (end_is_lanelet) {
            lanelet::ConstLanelet end = map->laneletLayer.get(end_id);
            tryPath(start, end);
            tryPath(start.invert(), end);
            tryPath(start, end.invert());
            tryPath(start.invert(), end.invert());
        } else {
            lanelet::ConstArea end = map->areaLayer.get(end_id);
            tryPath(start, end);
            tryPath(start.invert(), end);
        }
    } else {
        lanelet::ConstArea start = map->areaLayer.get(start_id);
        if (end_is_lanelet) {
            lanelet::ConstLanelet end = map->laneletLayer.get(end_id);
            tryPath(start, end);
            tryPath(start, end.invert());
        } else {
            lanelet::ConstArea end = map->areaLayer.get(end_id);
            tryPath(start, end);
        }
    }

    if (!bestPath) {
        RCLCPP_ERROR(rclcpp::get_logger("OptimalPathFinder"), "Optimal path not found.");
        throw std::runtime_error("Optimal path not found.");
    }

    RCLCPP_INFO(rclcpp::get_logger("OptimalPathFinder"), 
                "Optimal path found with shortest distance: %.2f meters.", shortestDistance);
    return bestPath.get();
}


PathResponse OptimalPathFinder::buildPathResponse(
    const lanelet::routing::LaneletOrAreaPath& path,
    const lanelet::traffic_rules::TrafficRules& amrTrafficRules) {

    PathResponse response;
    response.total_distance = calculatePathDistance(path);
    response.estimated_time = estimateTravelTime(path, amrTrafficRules);
    response.status = 0;  // Success

    RCLCPP_INFO(rclcpp::get_logger("OptimalPathFinder"), 
                "Building path response: Total distance = %.2f meters, Estimated time = %.2f seconds.",
                response.total_distance, response.estimated_time);

    for (const auto& element : path) {
        if (element.isLanelet()) {
            auto lanelet = static_cast<lanelet::ConstLanelet>(element);
            response.lanelet_ids.push_back(static_cast<int64_t>(lanelet.id()));
            response.is_inverted.push_back(lanelet.inverted());
            RCLCPP_INFO(rclcpp::get_logger("OptimalPathFinder"), 
                        "Lanelet ID %ld added to path, Inverted: %s", 
                        lanelet.id(), lanelet.inverted() ? "true" : "false");
        } else if (element.isArea()) {
            auto area = static_cast<lanelet::ConstArea>(element);
            response.lanelet_ids.push_back(static_cast<int64_t>(area.id()));
            response.is_inverted.push_back(false);  // Areas are not inverted
            RCLCPP_INFO(rclcpp::get_logger("OptimalPathFinder"), 
                        "Area ID %ld added to path.", area.id());
        }
    }

    response.message = "Optimal path successfully computed.";
    RCLCPP_INFO(rclcpp::get_logger("OptimalPathFinder"), "Path response built successfully.");
    return response;
}

NearestElement OptimalPathFinder::getNearestLaneletOrArea(
    const lanelet::LaneletMapPtr& map,
    const GPSPoint& gps_point,
    const GPSPoint& origin_point,
    double max_dist_threshold) {
    RCLCPP_INFO(rclcpp::get_logger("OptimalPathFinder"),
                "Getting nearest lanelet or area for GPS point (%.6f, %.6f).",
                gps_point.latitude, gps_point.longitude);

    lanelet::BasicPoint2d utm_point = convertGPSToUTM(gps_point, origin_point);

    // Get multiple nearest lanelets and areas
    auto nearest_lanelets = lanelet::geometry::findNearest(map->laneletLayer, utm_point, 5);
    auto nearest_areas = lanelet::geometry::findNearest(map->areaLayer, utm_point, 5);

    NearestElement nearest_element;
    double min_distance = std::numeric_limits<double>::max();

    // Check lanelets
    for (const auto& lanelet_pair : nearest_lanelets) {
        const auto& lanelet = lanelet_pair.second;
        if (lanelet.hasAttribute("amr_navigable") && lanelet.attribute("amr_navigable") == "yes") {
            if (lanelet_pair.first < min_distance) {
                min_distance = lanelet_pair.first;
                nearest_element.id = lanelet.id();
                nearest_element.isLanelet = true;
            }
        }
    }

    // Check areas
    for (const auto& area_pair : nearest_areas) {
        const auto& area = area_pair.second;
        if (area.hasAttribute("amr_navigable") && area.attribute("amr_navigable") == "yes") {
            if (area_pair.first < min_distance) {
                min_distance = area_pair.first;
                nearest_element.id = area.id();
                nearest_element.isLanelet = false;
            }
        }
    }

    // if (min_distance == std::numeric_limits<double>::max()) {
    //     RCLCPP_ERROR(rclcpp::get_logger("OptimalPathFinder"), "No nearby navigable lanelet or area found.");
    //     throw std::runtime_error("No nearby navigable lanelet or area found.");
    // }

    // Check if the nearest element is within the threshold distance
    if (min_distance > max_dist_threshold) {
        RCLCPP_ERROR(rclcpp::get_logger("OptimalPathFinder"),
                        "No nearby navigable lanelet or area found within the threshold distance.");
        throw std::runtime_error("No nearby navigable lanelet or area found within the threshold distance.");
    }

    RCLCPP_INFO(rclcpp::get_logger("OptimalPathFinder"),
                "Nearest %s found: ID = %ld at distance of %.2f",
                nearest_element.isLanelet ? "lanelet" : "area",
                nearest_element.id, min_distance);

    return nearest_element;
}

lanelet::BasicPoint2d OptimalPathFinder::convertGPSToUTM(const GPSPoint& gps_point, const GPSPoint& origin_point) {
    lanelet::projection::UtmProjector projector(lanelet::Origin({origin_point.latitude, origin_point.longitude}));
    lanelet::GPSPoint gps = {gps_point.latitude, gps_point.longitude, gps_point.altitude};

    lanelet::BasicPoint3d utm_point = projector.forward(gps);
    RCLCPP_INFO(rclcpp::get_logger("OptimalPathFinder"), 
                "Converted GPS (%.6f, %.6f) to UTM (%.2f, %.2f).",
                gps.lat, gps.lon, utm_point.x(), utm_point.y());
    return lanelet::BasicPoint2d(utm_point.x(), utm_point.y());
}

double OptimalPathFinder::calculatePathDistance(const lanelet::routing::LaneletOrAreaPath& path) {
    double total_distance = 0.0;

    for (size_t i = 0; i < path.size() - 1; ++i) {
        total_distance += getLength(path[i]) + getLength(path[i + 1]);
    }

    RCLCPP_INFO(rclcpp::get_logger("OptimalPathFinder"), 
                "Total path distance: %.2f meters.", total_distance);

    return total_distance;
}

double OptimalPathFinder::getLength(const lanelet::ConstLaneletOrArea& la) const {
    double length = 0.0;

    if (la.isLanelet()) {
        auto laneletOpt = la.lanelet();
        if (laneletOpt) {
            length = lanelet::geometry::approximatedLength2d(laneletOpt.get());
        }
    } else {
        auto areaOpt = la.area();
        if (areaOpt) {
            length = boost::geometry::perimeter(lanelet::utils::to2D(areaOpt.get().outerBoundPolygon()));
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("OptimalPathFinder"), 
                "Length of lanelet/area (ID %ld): %.2f meters.", la.id(), length);
    
    return length;
}

double OptimalPathFinder::estimateTravelTime(const lanelet::routing::LaneletOrAreaPath& path, 
                                             const lanelet::traffic_rules::TrafficRules& amrTrafficRules) {
    double total_time = 0.0;

    for (const auto& la : path) {
        total_time += getTravelTime(la, amrTrafficRules);
    }

    RCLCPP_INFO(rclcpp::get_logger("OptimalPathFinder"), 
                "Total estimated travel time: %.2f seconds.", total_time);

    return total_time;
}

double OptimalPathFinder::getTravelTime(const lanelet::ConstLaneletOrArea& la, 
                                        const lanelet::traffic_rules::TrafficRules& amrTrafficRules) const {
    double time = 0.0;

    if (la.isLanelet()) {
        auto laneletOpt = la.lanelet();
        if (laneletOpt) {
            double length = lanelet::geometry::approximatedLength2d(laneletOpt.get());
            lanelet::Velocity speedLimit = amrTrafficRules.speedLimit(laneletOpt.get()).speedLimit;
            time = length / speedLimit.value();
            RCLCPP_INFO(rclcpp::get_logger("OptimalPathFinder"), 
                        "Lanelet ID %ld: Length = %.2f meters, Speed Limit = %.2f m/s, Estimated time = %.2f seconds.",
                        la.id(), length, speedLimit.value(), time);
        }
    } else {
        auto areaOpt = la.area();
        if (areaOpt) {
            double perimeter = boost::geometry::perimeter(lanelet::utils::to2D(areaOpt.get().outerBoundPolygon()));
            lanelet::Velocity speedLimit = amrTrafficRules.speedLimit(areaOpt.get()).speedLimit;
            time = perimeter / speedLimit.value();
            RCLCPP_INFO(rclcpp::get_logger("OptimalPathFinder"), 
                        "Area ID %ld: Perimeter = %.2f meters, Speed Limit = %.2f m/s, Estimated time = %.2f seconds.",
                        la.id(), perimeter, speedLimit.value(), time);
        }
    }

    return time;
}

} // namespace amr_navigation
