#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
import lanelet2
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
            self.node.get_logger().info(f"Map loaded successfully with {len(self.map.laneletLayer)} lanelets")
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
                    (self.get_lanelet_by_id(start_lanelet_id, True), 
                     self.get_lanelet_by_id(end_lanelet_id, True))
                ]
                
                for start_l, end_l in combinations:
                    if start_l and end_l:
                        path = self.routing_graph.getRoute(start_l, end_l)
                        if path:
                            return path
            
            self.node.get_logger().warn(f"No path found between lanelets {start_lanelet_id} and {end_lanelet_id}")
            return None
        except Exception as e:
            self.node.get_logger().error(f"Failed to find path: {str(e)}")
            return None
        


#!/usr/bin/env python3

# filepath: /home/abhi/ros2_ws/src/AMR-Logistic-System/amr_trajectory_planner/amr_trajectory_planner/test_lanelet_map.py

import sys
import time
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("LaneletMapTest")

# Mock Node class for testing without ROS
class MockNode:
    def __init__(self):
        self.logger = logger
    
    def get_logger(self):
        return self.logger

# Import the LaneletMapManager class
sys.path.append('/home/abhi/ros2_ws/src/AMR-Logistic-System/amr_trajectory_planner/amr_trajectory_planner')
from lanelet_map_manager import LaneletMapManager

def main():
    """Test the LaneletMapManager without ROS2"""
    logger.info("Starting Lanelet2 Map Test")
    
    # Hardcoded values for testing
    map_path = "/home/abhi/ros2_ws/src/AMR-Logistic-System/laneletMaps/HS-SchmalkaldenPart1.osm"
    origin_lat = 50.7153508
    origin_lon = 10.4680332
    origin_alt = 0.0
    
    # Create a mock node
    mock_node = MockNode()
    
    # Create the map manager
    map_manager = LaneletMapManager(mock_node, map_path, origin_lat, origin_lon, origin_alt)
    
    # Load the map
    if not map_manager.load_map():
        logger.error("Failed to load map, exiting...")
        return
    
    # Build the routing graph
    if not map_manager.build_routing_graph():
        logger.error("Failed to build routing graph, exiting...")
        return
        
    # Check if map loaded correctly
    map = map_manager.map
    logger.info(f"Map loaded successfully with {len(map.laneletLayer)} lanelets")
    
    # Print the first few lanelet IDs for verification
    lanelet_ids = list(map.laneletLayer.keys())
    if lanelet_ids:
        logger.info(f"First 5 lanelet IDs: {lanelet_ids[:5]}")
        
        # Try to get detailed info about the first lanelet
        first_lanelet = map_manager.get_lanelet_by_id(lanelet_ids[0])
        if first_lanelet:
            left_bound_pts = len(first_lanelet.leftBound)
            right_bound_pts = len(first_lanelet.rightBound)
            logger.info(f"Lanelet {lanelet_ids[0]} has {left_bound_pts} left boundary points and {right_bound_pts} right boundary points")
            
            # Check if it's navigable by AMR
            is_navigable = first_lanelet.hasAttribute("amr_navigable") and first_lanelet.attribute("amr_navigable") == "yes"
            logger.info(f"Lanelet {lanelet_ids[0]} is {'navigable' if is_navigable else 'not navigable'} by AMR")
            
            # Get centerline and print some points
            centerline = first_lanelet.centerline
            if centerline:
                logger.info(f"Centerline has {len(centerline)} points")
                if len(centerline) > 0:
                    logger.info(f"First centerline point: ({centerline[0].x}, {centerline[0].y})")
    
    # Try to find a path between two lanelets if we have at least 2
    if len(lanelet_ids) >= 2:
        start_id = lanelet_ids[0]
        end_id = lanelet_ids[-1]
        logger.info(f"Attempting to find path from lanelet {start_id} to {end_id}...")
        path = map_manager.find_path(start_id, end_id)
        if path:
            logger.info(f"Path found! Length: {len(path.shortestPath())}")
            for idx, lanelet in enumerate(path.shortestPath()):
                logger.info(f"Step {idx+1}: Lanelet {lanelet.id}")
        else:
            logger.info("No path found. Trying with more lanelet combinations...")
            
            # Try all combinations of the first few lanelets
            sample_size = min(5, len(lanelet_ids))
            for i in range(sample_size):
                for j in range(i+1, sample_size):
                    start_id = lanelet_ids[i]
                    end_id = lanelet_ids[j]
                    logger.info(f"Testing path from {start_id} to {end_id}...")
                    path = map_manager.find_path(start_id, end_id)
                    if path:
                        logger.info(f"Path found between {start_id} and {end_id}!")
                        # Print the path
                        logger.info(f"Path contains {len(path.shortestPath())} lanelets:")
                        for k, l in enumerate(path.shortestPath()):
                            logger.info(f"  {k+1}. Lanelet {l.id}")
                        break
                if path:
                    break
            
            if not path:
                logger.info("Could not find a path between any tested lanelet pairs.")
    
    logger.info("Map testing complete")

if __name__ == '__main__':
    main()