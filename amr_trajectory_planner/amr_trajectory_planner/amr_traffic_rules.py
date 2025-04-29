import lanelet2
from lanelet2.core import Velocity
from lanelet2.traffic_rules import TrafficRules, SpeedLimitInformation

class AmrTrafficRules(TrafficRules):
    """
    Custom traffic rules implementation for Autonomous Mobile Robots (AMR)
    
    This is a direct Python port of the C++ AmrTrafficRules class
    """
    
    def __init__(self):
        """Initialize with a default configuration for AMR"""
        # Create a configuration object
        config = TrafficRules.Configuration()
        config.location = "Germany"
        config.participant = "vehicle"
        super().__init__(config)
    
    def canPass(self, obj, to_obj=None):
        """
        Check if an element can be passed by the AMR
        
        Args:
            obj: First lanelet or area
            to_obj: Second lanelet or area (optional)
            
        Returns:
            bool: True if passing is allowed
        """
        # Single element case
        if to_obj is None:
            # Check if it's a lanelet
            if hasattr(obj, 'isLanelet') and obj.isLanelet():
                return obj.hasAttribute("amr_navigable") and obj.attribute("amr_navigable") == "yes"
            # Otherwise it's an area
            return obj.hasAttribute("amr_navigable") and obj.attribute("amr_navigable") == "yes"
        
        # Lanelet to lanelet case
        if hasattr(obj, 'isLanelet') and obj.isLanelet() and \
           hasattr(to_obj, 'isLanelet') and to_obj.isLanelet():
            # Special case: if they are the same lanelet or inverted versions
            if obj.id == to_obj.id or obj.id == to_obj.invert().id:
                return True
            # Otherwise check if both are navigable
            return self.canPass(obj) and self.canPass(to_obj)
        
        # All other combinations (lanelet-area, area-lanelet, area-area)
        return self.canPass(obj) and self.canPass(to_obj)
    
    def canChangeLane(self, from_lanelet, to_lanelet):
        """AMRs don't change lanes"""
        return False
    
    def speedLimit(self, element):
        """Get the speed limit for a lanelet or area"""
        # Create a 30 km/h velocity
        speed = Velocity(30.0, Velocity.Attribute.KMH)
        return SpeedLimitInformation(speed, True)
    
    def isOneWay(self, lanelet):
        """Check if a lanelet is one-way"""
        if lanelet.hasAttribute("one_way") and lanelet.attribute("one_way") == "yes":
            return not lanelet.hasAttribute("bidirectional") or lanelet.attribute("bidirectional") != "yes"
        return False
    
    def hasDynamicRules(self, lanelet):
        """Check if a lanelet has dynamic rules"""
        return False