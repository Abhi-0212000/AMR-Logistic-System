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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "amr_utils_python/coordinate_transforms.hpp"

namespace py = pybind11;

PYBIND11_MODULE(coordinate_transforms_py, m)
{
  // Bind GPSPoint struct
  py::class_<amr_local_planner::GPSPoint>(m, "GPSPoint")
  .def(py::init<>())                          // Default constructor
  .def(py::init<double, double, double>())    // Constructor with parameters
  .def_readwrite("latitude", &amr_local_planner::GPSPoint::latitude)
  .def_readwrite("longitude", &amr_local_planner::GPSPoint::longitude)
  .def_readwrite("altitude", &amr_local_planner::GPSPoint::altitude);

  // Bind LocalPoint struct
  py::class_<amr_local_planner::LocalPoint>(m, "LocalPoint")
  .def(py::init<>())                                              // Default constructor
  .def(py::init<double, double>(), py::arg("x"), py::arg("y"))    // Constructor with parameters
  .def_readwrite("x", &amr_local_planner::LocalPoint::x)
  .def_readwrite("y", &amr_local_planner::LocalPoint::y);

  // Bind CoordinateTransforms class
  py::class_<amr_local_planner::CoordinateTransforms>(m, "CoordinateTransforms")
  .def(py::init<const amr_local_planner::GPSPoint &>())
  .def("gps_to_local", &amr_local_planner::CoordinateTransforms::gpsToLocal)
  .def("local_to_gps", &amr_local_planner::CoordinateTransforms::localToGPS);
}
