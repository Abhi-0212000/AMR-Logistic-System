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

#ifndef AMR_NAVIGATION_SYSTEM__UTILS__COMMON_UTILS_HPP_
#define AMR_NAVIGATION_SYSTEM__UTILS__COMMON_UTILS_HPP_

#include <lanelet2_core/primitives/Lanelet.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace amr_navigation
{

// Struct to hold GPS point data
struct GPSPoint
{
  double latitude;
  double longitude;
  double altitude;

  // Constructor to initialize the values
  explicit GPSPoint(double lat = 0.0, double lon = 0.0, double alt = 0.0)
  : latitude(lat), longitude(lon), altitude(alt)
  {
  }
};

struct PathResponse
{
  std::vector<int64_t> lanelet_ids;
  std::vector<bool> is_inverted;  // New field for inversion info
  double total_distance;
  double estimated_time;
  std::uint8_t status;
  std::string message;
};

struct NearestElement
{
  lanelet::Id id;  // ID of the nearest lanelet or area
  bool isLanelet;  // Flag to indicate if it is a lanelet (true) or area (false)
};

struct BoundingBox
{
  double min_lat, min_lon, max_lat, max_lon;
};

inline bool isWithinBounds(const GPSPoint & point, const BoundingBox & bbox)
{
  return point.latitude >= bbox.min_lat && point.latitude <= bbox.max_lat &&
         point.longitude >= bbox.min_lon && point.longitude <= bbox.max_lon;
}

bool checkFileExists(const std::string & filePath);
void createDirectory(const std::string & dirPath);
std::string expandTilde(const std::string & path);
std::string getDefaultDebugPath();

}  // namespace amr_navigation

#endif  // AMR_NAVIGATION_SYSTEM__UTILS__COMMON_UTILS_HPP_
