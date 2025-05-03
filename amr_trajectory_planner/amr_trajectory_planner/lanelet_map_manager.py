#!/usr/bin/env python3

# from lanelet2.core import BasicPoint3d, GPSPoint, Origin
from lanelet2.projection import UtmProjector
from lanelet2.io import load as load_lanelet, Origin
from lanelet2.routing import RoutingGraph

# from amr_trajectory_planner.amr_traffic_rules import AmrTrafficRules, create_amr_traffic_rules
from .load_amr_traffic_rules import get_amr_traffic_rules
from amr_utils_python.coordinate_transforms_py import GPSPoint


class LaneletMapManager:
    def __init__(self, node, map_path, map_origin: GPSPoint):
        self.node = node
        self.map_path = map_path
        self.origin = Origin(map_origin.latitude, map_origin.longitude, map_origin.altitude)
        self.map = None
        self.routing_graph = None
        self.traffic_rules = None

    def load_map(self):
        """Load the Lanelet2 map file"""
        try:
            self.node.get_logger().info(f"Loading map from {self.map_path}")
            projector = UtmProjector(self.origin)
            self.map = load_lanelet(self.map_path, projector)
            self.node.get_logger().info(
                f"Map loaded successfully with {len(self.map.laneletLayer)} lanelets"
            )
            return True
        except Exception as e:
            self.node.get_logger().error(f"Failed to load map: {str(e)}")
            return False

    def build_routing_graph(self):
        """Build a routing graph from the loaded map with custom AMR traffic rules"""
        if not self.map:
            self.node.get_logger().error("Cannot build routing graph: Map not loaded")
            return False

        try:
            # Create our custom traffic rules directly
            self.traffic_rules = get_amr_traffic_rules()

            # Build routing graph with our custom rules
            self.routing_graph = RoutingGraph(self.map, self.traffic_rules)
            self.node.get_logger().info("Routing graph built with AMR traffic rules successfully")
            return True
        except Exception as e:
            self.node.get_logger().error(f"Failed to build routing graph: {str(e)}")
            import traceback

            self.node.get_logger().error(traceback.format_exc())
            return False

    def get_lanelet_by_id(self, lanelet_id, inverted=False):
        """Get a lanelet by its ID, optionally inverted"""
        if not self.map:
            self.node.get_logger().error("Cannot get lanelet: Map not loaded")
            return None

        try:
            lanelet = self.map.laneletLayer.get(lanelet_id)
            if inverted:
                return lanelet.invert()
            return lanelet
        except Exception as e:
            self.node.get_logger().error(f"Failed to get lanelet {lanelet_id}: {str(e)}")
            return None
