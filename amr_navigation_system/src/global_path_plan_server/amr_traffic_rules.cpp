#include "amr_navigation_system/global_path_plan_server/amr_traffic_rules.hpp"
#include <boost/optional.hpp>  // Include for boost::optional (used in Lanelet2)

// Namespace lanelet to work with Lanelet2 library
namespace lanelet {
// Nested namespace for traffic rules related classes and functions
namespace traffic_rules {

// Constructor: Initializes AmrTrafficRules by calling the base class constructor with default configuration
AmrTrafficRules::AmrTrafficRules() : TrafficRules(Configuration()) {}

// canPass function for Lanelet objects
// This checks if a lanelet has an attribute "amr_navigable" set to "yes"
// If so, the function returns true, indicating the lanelet can be passed by the AMR.
bool AmrTrafficRules::canPass(const lanelet::ConstLanelet& lanelet) const {
    return lanelet.hasAttribute("amr_navigable") &&
           lanelet.attribute("amr_navigable").value() == "yes";
}

// canPass function for Area objects
// This checks if an area has an attribute "amr_navigable" set to "yes"
// If so, the function returns true, indicating the area can be passed by the AMR.
bool AmrTrafficRules::canPass(const lanelet::ConstArea& area) const {
    return area.hasAttribute("amr_navigable") &&
           area.attribute("amr_navigable").value() == "yes";
}

// canPass function for two Lanelets
// Checks if both 'from' and 'to' lanelets are navigable by calling the single lanelet canPass function.
// Special case: If the 'from' and 'to' lanelets are identical (including one being inverted), this function allows passing.
bool AmrTrafficRules::canPass(const lanelet::ConstLanelet& from, const lanelet::ConstLanelet& to) const {
    if (from.id() == to.id() || from.id() == to.invert().id()) {
        return true;  // No conflict between a lanelet and its inverted version
    }
    return canPass(from) && canPass(to);
}

// canPass function from a Lanelet to an Area
// This checks if both the starting lanelet and the destination area are navigable.
bool AmrTrafficRules::canPass(const lanelet::ConstLanelet& from, const lanelet::ConstArea& to) const {
    return canPass(from) && canPass(to);
}

// canPass function from an Area to a Lanelet
// This checks if both the starting area and the destination lanelet are navigable.
bool AmrTrafficRules::canPass(const lanelet::ConstArea& from, const lanelet::ConstLanelet& to) const {
    return canPass(from) && canPass(to);
}

// canPass function from an Area to another Area
// This checks if both areas (from and to) are navigable by calling the respective canPass function.
bool AmrTrafficRules::canPass(const lanelet::ConstArea& from, const lanelet::ConstArea& to) const {
    return canPass(from) && canPass(to);
}

// canChangeLane function
// This function returns false for all cases, indicating that AMRs (Autonomous Mobile Robots) do not change lanes.
bool AmrTrafficRules::canChangeLane(const lanelet::ConstLanelet& /*from*/, const lanelet::ConstLanelet& /*to*/) const {
    return false;  // AMRs typically don't change lanes
}

// isOneWay function
// This function checks if a given lanelet is marked as "one_way" with the value "yes".
// Additionally, it checks if the lanelet is not marked as "bidirectional".
bool AmrTrafficRules::isOneWay(const lanelet::ConstLanelet& lanelet) const {
    if (lanelet.hasAttribute("one_way") && lanelet.attribute("one_way").value() == "yes") {
        return !lanelet.hasAttribute("bidirectional") || lanelet.attribute("bidirectional").value() != "yes";
    }
    return false;
}

// speedLimit function for Lanelets
// This function returns a default speed limit of 30 km/h for any lanelet.
// The limit is marked as mandatory (true).
SpeedLimitInformation AmrTrafficRules::speedLimit([[maybe_unused]] const lanelet::ConstLanelet& lanelet) const {
    using namespace lanelet::units::literals; 
    Velocity speed = 30.0_kmh;  // Define speed limit for lanelets

    return SpeedLimitInformation{speed, true};  // true for mandatory speed limit
}

// speedLimit function for Areas
// This function returns a default speed limit of 30 km/h for any area.
// The limit is marked as mandatory (true).
SpeedLimitInformation AmrTrafficRules::speedLimit([[maybe_unused]] const lanelet::ConstArea& area) const {
    using namespace lanelet::units::literals;
    Velocity speed = 30.0_kmh;  // Same speed limit for areas

    return SpeedLimitInformation{speed, true};  // true for mandatory speed limit
}

// hasDynamicRules function
// This function returns false, indicating there are no dynamic traffic rules in the current system.
bool AmrTrafficRules::hasDynamicRules([[maybe_unused]] const lanelet::ConstLanelet& lanelet) const {
    return false;  // Assume no dynamic rules for simplicity
}

}  // namespace traffic_rules
}  // namespace lanelet
