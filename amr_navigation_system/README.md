# amr_navigation_system

The `amr_navigation_system` is a ROS2 package developed for autonomous navigation of mobile robots using the Lanelet2 library. This system allows robots to plan global paths based on a Lanelet2 map with custom modifications. It is intended for internal use at Hochschule Schmalkalden for R&D, teaching, and research purposes.

This package is independent of the college's core system but is designed to integrate with the overall AMR system. It will also serve as a foundation for future publications and further enhancements.


## Features
- Global path planning using Lanelet2.
- GPS filtering and nearest-lanelet detection.
- Map boundary validation and non-navigable area exclusion.
- Scalable for large-scale environments and custom maps.



## Installation
### Dependencies
- ROS2 Humble
- Lanelet2 library

Install dependencies using:
```bash
sudo apt install ros-foxy-lanelet2
sudo apt install libspdlog-dev


