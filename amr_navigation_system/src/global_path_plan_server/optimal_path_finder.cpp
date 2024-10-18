#include "amr_navigation_system/global_path_plan_server/optimal_path_finder.hpp"
#include <limits>
#include <stdexcept> // For runtime_error

namespace amr_navigation {

// Constructor
OptimalPathFinder::OptimalPathFinder(std::shared_ptr<amr_logging::NodeLogger> logger)
    : logger_(logger) {}

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
            
            logger_->log_debug(fmt::format("Trying path with IDs: {}.", path_ids.str().c_str()), __FILE__, __LINE__, __FUNCTION__);

            // Calculate and log the path distance
            double distance = calculatePathDistance(path.get());

        logger_->log_debug(fmt::format("Path distance: {} meters.", distance), __FILE__, __LINE__, __FUNCTION__);

            if (distance < shortestDistance) {
                bestPath = path;
                shortestDistance = distance;
                logger_->log_debug(fmt::format("New shortest path found with distance: {} meters.", shortestDistance), __FILE__, __LINE__, __FUNCTION__);
            }
        } else {
            logger_->log_warn(fmt::format("No valid path found from start ID {} to end ID {}.", start.id(), end.id()), __FILE__, __LINE__, __FUNCTION__);
        }
    };

    logger_->log_debug(fmt::format("Finding optimal path from start ID {} to end ID {}...", start_id, end_id), __FILE__, __LINE__, __FUNCTION__);

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
        logger_->log_error("Optimal path not found.", __FILE__, __LINE__, __FUNCTION__);
        throw std::runtime_error("Optimal path not found.");
    }
    logger_->log_info(fmt::format("Optimal path found with shortest distance: {} meters.", shortestDistance), __FILE__, __LINE__, __FUNCTION__);
    return bestPath.get();
}


PathResponse OptimalPathFinder::buildPathResponse(
    const lanelet::routing::LaneletOrAreaPath& path,
    const lanelet::traffic_rules::TrafficRules& amrTrafficRules) {

    PathResponse response;
    response.total_distance = calculatePathDistance(path);
    response.estimated_time = estimateTravelTime(path, amrTrafficRules);
    response.status = 0;  // Success

    logger_->log_info(fmt::format("Building path response: Total distance = {:.2f} meters, Estimated time = {:.2f} seconds.", response.total_distance, response.estimated_time), __FILE__, __LINE__, __FUNCTION__);

    for (const auto& element : path) {
        if (element.isLanelet()) {
            auto lanelet = static_cast<lanelet::ConstLanelet>(element);
            response.lanelet_ids.push_back(static_cast<int64_t>(lanelet.id()));
            response.is_inverted.push_back(lanelet.inverted());
            logger_->log_debug(fmt::format("Lanelet ID {} added to path, Inverted: {}", lanelet.id(), lanelet.inverted() ? "true" : "false"), __FILE__, __LINE__, __FUNCTION__);
        } else if (element.isArea()) {
            auto area = static_cast<lanelet::ConstArea>(element);
            response.lanelet_ids.push_back(static_cast<int64_t>(area.id()));
            response.is_inverted.push_back(false);  // Areas are not inverted
            logger_->log_debug(fmt::format("Area ID {} added to path.", area.id()), __FILE__, __LINE__, __FUNCTION__);
        }
    }

    response.message = "Optimal path successfully computed.";
    logger_->log_info("Path response built successfully.", __FILE__, __LINE__, __FUNCTION__);
    return response;
}

NearestElement OptimalPathFinder::getNearestLaneletOrArea(
    const lanelet::LaneletMapPtr& map,
    const GPSPoint& gps_point,
    const GPSPoint& origin_point,
    double max_dist_threshold) {
    logger_->log_debug(fmt::format("Getting nearest lanelet or area for GPS point ({:.6f}, {:.6f}).",
                gps_point.latitude, gps_point.longitude), __FILE__, __LINE__, __FUNCTION__);

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
        logger_->log_error("No nearby navigable lanelet or area found within the threshold distance.", __FILE__, __LINE__, __FUNCTION__);
        throw std::runtime_error("No nearby navigable lanelet or area found within the threshold distance.");
    }
    logger_->log_debug(fmt::format("Nearest {} found: ID = {} at distance of {:.2f}",
                nearest_element.isLanelet ? "lanelet" : "area",
                nearest_element.id, min_distance), __FILE__, __LINE__, __FUNCTION__);

    return nearest_element;
}

lanelet::BasicPoint2d OptimalPathFinder::convertGPSToUTM(const GPSPoint& gps_point, const GPSPoint& origin_point) {
    lanelet::projection::UtmProjector projector(lanelet::Origin({origin_point.latitude, origin_point.longitude}));
    lanelet::GPSPoint gps = {gps_point.latitude, gps_point.longitude, gps_point.altitude};

    lanelet::BasicPoint3d utm_point = projector.forward(gps);
    logger_->log_debug(fmt::format("Converted GPS ({:.6f}, {:.6f}) to UTM ({:.2f}, {:.2f}).",
                gps.lat, gps.lon, utm_point.x(), utm_point.y()), __FILE__, __LINE__, __FUNCTION__);
    return lanelet::BasicPoint2d(utm_point.x(), utm_point.y());
}

double OptimalPathFinder::calculatePathDistance(const lanelet::routing::LaneletOrAreaPath& path) {
    double total_distance = 0.0;

    for (size_t i = 0; i < path.size() - 1; ++i) {
        total_distance += getLength(path[i]) + getLength(path[i + 1]);
    }
    logger_->log_debug(fmt::format("Total path distance: {:.2f} meters.", total_distance), __FILE__, __LINE__, __FUNCTION__);

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
    logger_->log_debug(fmt::format("Length of lanelet/area (ID {}): {:.2f} meters.", la.id(), length), __FILE__, __LINE__, __FUNCTION__);
    
    return length;
}

double OptimalPathFinder::estimateTravelTime(const lanelet::routing::LaneletOrAreaPath& path, 
                                             const lanelet::traffic_rules::TrafficRules& amrTrafficRules) {
    double total_time = 0.0;

    for (const auto& la : path) {
        total_time += getTravelTime(la, amrTrafficRules);
    }
    logger_->log_debug(fmt::format("Total estimated travel time: {:.2f} seconds.", total_time), __FILE__, __LINE__, __FUNCTION__);

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
            logger_->log_debug(fmt::format("Lanelet ID {}: Length = {:.2f} meters, Speed Limit = {:.2f} m/s, Estimated time = {:.2f} seconds.",
                        la.id(), length, speedLimit.value(), time), __FILE__, __LINE__, __FUNCTION__);
        }
    } else {
        auto areaOpt = la.area();
        if (areaOpt) {
            double perimeter = boost::geometry::perimeter(lanelet::utils::to2D(areaOpt.get().outerBoundPolygon()));
            lanelet::Velocity speedLimit = amrTrafficRules.speedLimit(areaOpt.get()).speedLimit;
            time = perimeter / speedLimit.value();
            logger_->log_debug(fmt::format("Area ID {}: Perimeter = {:.2f} meters, Speed Limit = {:.2f} m/s, Estimated time = {:.2f} seconds.",
                        la.id(), perimeter, speedLimit.value(), time), __FILE__, __LINE__, __FUNCTION__);
        }
    }

    return time;
}

} // namespace amr_navigation
