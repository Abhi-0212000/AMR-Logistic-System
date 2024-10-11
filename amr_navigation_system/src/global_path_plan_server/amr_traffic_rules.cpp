#include "amr_navigation_system/global_path_plan_server/amr_traffic_rules.hpp"
#include <lanelet2_core/utility/Units.h>
#include <boost/optional.hpp>  // Include this for boost::optional


namespace lanelet {
namespace traffic_rules {

AmrTrafficRules::AmrTrafficRules() : TrafficRules(Configuration()) {}

bool AmrTrafficRules::canPass(const lanelet::ConstLanelet& lanelet) const {
    return lanelet.hasAttribute("amr_navigable") &&
           lanelet.attribute("amr_navigable").value() == "yes";
}

bool AmrTrafficRules::canPass(const lanelet::ConstArea& area) const {
    return area.hasAttribute("amr_navigable") &&
           area.attribute("amr_navigable").value() == "yes";
}

// bool AmrTrafficRules::canPass(const lanelet::ConstLanelet& from, const lanelet::ConstLanelet& to) const {
//     return canPass(from) && canPass(to);
// }

bool AmrTrafficRules::canPass(const lanelet::ConstLanelet& from, const lanelet::ConstLanelet& to) const {
    // Check if the 'from' and 'to' lanelets are identical, even if one is inverted
    if (from.id() == to.id() || from.id() == to.invert().id()) {
        return true;  // No conflict between a lanelet and its inverted version
    }
    return canPass(from) && canPass(to);
}


bool AmrTrafficRules::canPass(const lanelet::ConstLanelet& from, const lanelet::ConstArea& to) const {
    return canPass(from) && canPass(to);
}

bool AmrTrafficRules::canPass(const lanelet::ConstArea& from, const lanelet::ConstLanelet& to) const {
    return canPass(from) && canPass(to);
}

bool AmrTrafficRules::canPass(const lanelet::ConstArea& from, const lanelet::ConstArea& to) const {
    return canPass(from) && canPass(to);
}

bool AmrTrafficRules::canChangeLane(const lanelet::ConstLanelet& /*from*/, const lanelet::ConstLanelet& /*to*/) const {
    return false;  // AMRs typically don't change lanes
}

bool AmrTrafficRules::isOneWay(const lanelet::ConstLanelet& lanelet) const {
    if (lanelet.hasAttribute("one_way") && lanelet.attribute("one_way").value() == "yes") {
        return !lanelet.hasAttribute("bidirectional") || lanelet.attribute("bidirectional").value() != "yes";
    }
    return false;
}

SpeedLimitInformation AmrTrafficRules::speedLimit([[maybe_unused]] const lanelet::ConstLanelet& lanelet) const {
    using namespace lanelet::units::literals; 
    Velocity speed = 30.0_kmh;  // Define speed

    return SpeedLimitInformation{speed, true};  // true for mandatory
}

SpeedLimitInformation AmrTrafficRules::speedLimit([[maybe_unused]] const lanelet::ConstArea& area) const {
    using namespace lanelet::units::literals;
    Velocity speed = 30.0_kmh;  // Same for area

    return SpeedLimitInformation{speed, true};  // true for mandatory
}

bool AmrTrafficRules::hasDynamicRules([[maybe_unused]] const lanelet::ConstLanelet& lanelet) const {
    return false;  // Assume there are no dynamic rules
}


}  // namespace traffic_rules
}  // namespace lanelet
