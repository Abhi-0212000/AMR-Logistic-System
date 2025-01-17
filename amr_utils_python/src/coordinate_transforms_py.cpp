// coordinate_transforms_py.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "amr_utils_python/coordinate_transforms.hpp"

namespace py = pybind11;

PYBIND11_MODULE(coordinate_transforms_py, m) {
    using namespace amr_local_planner;
    
    // Bind GPSPoint struct
    py::class_<GPSPoint>(m, "GPSPoint")
        .def(py::init<>())  // Default constructor
        .def(py::init<double, double, double>())  // Constructor with parameters
        .def_readwrite("latitude", &GPSPoint::latitude)
        .def_readwrite("longitude", &GPSPoint::longitude)
        .def_readwrite("altitude", &GPSPoint::altitude);

    // Bind LocalPoint struct
    py::class_<LocalPoint>(m, "LocalPoint")
        .def(py::init<>())  // Default constructor
        .def(py::init<double, double>(), py::arg("x"), py::arg("y"))  // Constructor with parameters
        .def_readwrite("x", &LocalPoint::x)
        .def_readwrite("y", &LocalPoint::y);

    // Bind CoordinateTransforms class
    py::class_<CoordinateTransforms>(m, "CoordinateTransforms")
        .def(py::init<const GPSPoint&>())
        .def("gps_to_local", &CoordinateTransforms::gpsToLocal)
        .def("local_to_gps", &CoordinateTransforms::localToGPS);
}