#pragma once
#include <GeographicLib/UTMUPS.hpp>
#include <utility>
#include <stdexcept>

namespace amr_local_planner {

// Data structure for GPS points
struct GPSPoint {
    double latitude;
    double longitude;
    double altitude;

    // Default constructor
    GPSPoint() = default;

    // Constructor with individual parameters
    GPSPoint(double lat, double lon, double alt = 0.0) 
        : latitude(lat), longitude(lon), altitude(alt) {}
};

// Data structure for UTM points
struct UTMPoint {
    double x;      // Easting (meters)
    double y;      // Northing (meters)
    int zone;      // UTM zone number
    bool northp;   // Hemisphere: true for Northern, false for Southern

    // Default constructor
    UTMPoint() = default;

    // Constructor with parameters
    UTMPoint(double easting, double northing, int utm_zone, bool is_north = true)
        : x(easting), y(northing), zone(utm_zone), northp(is_north) {}
};

// Data structure for local points (relative to origin)
struct LocalPoint {
    double x;  // Local x position (relative to origin)
    double y;  // Local y position (relative to origin)

    // Default constructor
    LocalPoint() = default;

    // Constructor with parameters
    LocalPoint(double x_pos, double y_pos) 
        : x(x_pos), y(y_pos) {}
};

// Coordinate transformation class
class CoordinateTransforms {
public:
    // Constructor takes GPS origin and calculates its UTM equivalent for efficient future conversions
    explicit CoordinateTransforms(const GPSPoint& origin) {
        try {
            // Convert origin to UTM coordinates (stores the UTM zone and offsets for transformations)
            /*
                UTMUPS::Forward requires you to provide only the latitude and longitude as inputs. 
                The other four parameters (zone, northp, utm_x, and utm_y) act as output parameters.
                Purpose of Each Parameter After the Call:
                    zone: Assigned the UTM zone number for the input latitude and longitude.
                    northp: Assigned true if the latitude is in the Northern Hemisphere; otherwise, false.
                    utm_x and utm_y: Assigned the calculated easting and northing UTM coordinates, respectively.
            */
            GeographicLib::UTMUPS::Forward(
                origin.latitude,
                origin.longitude,
                origin_utm_.zone,         // UTM zone derived from origin coordinates
                origin_utm_.northp,       // Hemisphere derived from origin coordinates
                origin_utm_.x,            // Easting in UTM coordinates
                origin_utm_.y             // Northing in UTM coordinates
            );
        } catch (const GeographicLib::GeographicErr& e) {
            throw std::runtime_error("Error initializing UTM origin: " + std::string(e.what()));
        }
    }

    // Convert GPS to local coordinates (relative to origin)
    LocalPoint gpsToLocal(const GPSPoint& gps) const {
        int zone;
        bool northp;
        double utm_x, utm_y;

        try {
            // Convert GPS to UTM coordinates
            GeographicLib::UTMUPS::Forward(
                gps.latitude,
                gps.longitude,
                zone,
                northp,
                utm_x,
                utm_y
            );
        } catch (const GeographicLib::GeographicErr& e) {
            throw std::runtime_error("Error in gpsToLocal conversion: " + std::string(e.what()));
        }

        // Adjust UTM coordinates if GPS point is in a different zone or hemisphere
        if (zone != origin_utm_.zone || northp != origin_utm_.northp) {
            try {
                double transferred_x, transferred_y;
                int transferred_zone;
                GeographicLib::UTMUPS::Transfer(
                    zone,
                    northp,
                    utm_x,
                    utm_y,
                    origin_utm_.zone,
                    origin_utm_.northp,
                    transferred_x,
                    transferred_y,
                    transferred_zone
                );
                utm_x = transferred_x;
                utm_y = transferred_y;
            } catch (const GeographicLib::GeographicErr& e) {
                throw std::runtime_error("Error in zone transfer during gpsToLocal: " + std::string(e.what()));
            }
        }

        // Calculate local position relative to the origin
        return LocalPoint{
            utm_x - origin_utm_.x,
            utm_y - origin_utm_.y
        };
    }

    // Convert local coordinates back to GPS
    GPSPoint localToGPS(const LocalPoint& local) const {
        // Apply origin offsets to retrieve UTM position
        double utm_x = local.x + origin_utm_.x;
        double utm_y = local.y + origin_utm_.y;

        GPSPoint gps;
        try {
            // Convert UTM coordinates back to GPS
            GeographicLib::UTMUPS::Reverse(
                origin_utm_.zone,
                origin_utm_.northp,
                utm_x,
                utm_y,
                gps.latitude,
                gps.longitude
            );
        } catch (const GeographicLib::GeographicErr& e) {
            throw std::runtime_error("Error in localToGPS conversion: " + std::string(e.what()));
        }
        gps.altitude = 0.0;  // Default altitude
        return gps;
    }

private:
    UTMPoint origin_utm_;  // Stores UTM equivalent of GPS origin for future transformations
};

} // namespace amr_local_planner
