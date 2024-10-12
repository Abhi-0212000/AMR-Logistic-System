#ifndef COMMON_UTILS_HPP
#define COMMON_UTILS_HPP

#include <vector>
#include <string>
#include <filesystem>
#include <cstdint>
#include <lanelet2_core/primitives/Lanelet.h>

namespace amr_navigation {

// Struct to hold GPS point data
struct GPSPoint {
    double latitude;
    double longitude;
    double altitude;

    // Constructor to initialize the values
    GPSPoint(double lat = 0.0, double lon = 0.0, double alt = 0.0)
        : latitude(lat), longitude(lon), altitude(alt) {}
};

struct PathResponse {
    std::vector<int64_t> lanelet_ids;
    std::vector<bool> is_inverted; // New field for inversion info
    double total_distance;
    double estimated_time;
    std::uint8_t status;
    std::string message;
};

struct NearestElement {
    lanelet::Id id; // ID of the nearest lanelet or area
    bool isLanelet; // Flag to indicate if it is a lanelet (true) or area (false)
};

struct BoundingBox {
        double min_lat, min_lon, max_lat, max_lon;
    };

inline bool isWithinBounds(const GPSPoint& point, const BoundingBox& bbox) {
    return point.latitude >= bbox.min_lat && point.latitude <= bbox.max_lat &&
           point.longitude >= bbox.min_lon && point.longitude <= bbox.max_lon;
}

bool checkFileExists(const std::string& filePath);
void createDirectory(const std::string& dirPath);
std::string getDefaultDebugPath();

}  // namespace amr_navigation

#endif  // COMMON_UTILS_HPP