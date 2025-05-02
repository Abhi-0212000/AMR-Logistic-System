#!/usr/bin/env python3

# from lanelet2.core import BasicPoint3d, GPSPoint, Origin
from lanelet2.projection import UtmProjector
from lanelet2.io import load as load_lanelet, Origin
from lanelet2.routing import RoutingGraph

# from amr_trajectory_planner.amr_traffic_rules import AmrTrafficRules, create_amr_traffic_rules
from amr_trajectory_planner.load_amr_traffic_rules import get_amr_traffic_rules


class LaneletMapManager:
    def __init__(self, node, map_path, origin_lat, origin_lon, origin_alt=0.0):
        self.node = node
        self.map_path = map_path
        self.origin = Origin(origin_lat, origin_lon, origin_alt)
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

    def find_path(self, start_lanelet_id, end_lanelet_id, try_inverted=True):
        """Find a path between two lanelets, optionally trying inverted versions"""
        if not self.routing_graph:
            self.node.get_logger().error("Cannot find path: Routing graph not built")
            return None

        try:
            # Try with original orientation
            start = self.get_lanelet_by_id(start_lanelet_id)
            end = self.get_lanelet_by_id(end_lanelet_id)

            if start and end:
                path = self.routing_graph.getRoute(start, end)
                if path:
                    return path

            # If requested and initial attempt failed, try with inverted lanelets
            if try_inverted:
                combinations = [
                    (start, self.get_lanelet_by_id(end_lanelet_id, True)),
                    (self.get_lanelet_by_id(start_lanelet_id, True), end),
                    (
                        self.get_lanelet_by_id(start_lanelet_id, True),
                        self.get_lanelet_by_id(end_lanelet_id, True),
                    ),
                ]

                for start_l, end_l in combinations:
                    if start_l and end_l:
                        path = self.routing_graph.getRoute(start_l, end_l)
                        if path:
                            return path

            self.node.get_logger().warn(
                f"No path found between lanelets {start_lanelet_id} and {end_lanelet_id}"
            )
            return None
        except Exception as e:
            self.node.get_logger().error(f"Failed to find path: {str(e)}")
            return None
