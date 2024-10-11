#ifndef AMR_TRAFFIC_RULES_HPP
#define AMR_TRAFFIC_RULES_HPP

#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/utility/Units.h>

namespace lanelet {
namespace traffic_rules {

class AmrTrafficRules : public TrafficRules {
public:
    AmrTrafficRules();

    bool canPass(const lanelet::ConstLanelet& lanelet) const override;
    bool canPass(const lanelet::ConstArea& area) const override;
    bool canPass(const lanelet::ConstLanelet& from, const lanelet::ConstLanelet& to) const override;
    bool canPass(const lanelet::ConstLanelet& from, const lanelet::ConstArea& to) const override;
    bool canPass(const lanelet::ConstArea& from, const lanelet::ConstLanelet& to) const override;
    bool canPass(const lanelet::ConstArea& from, const lanelet::ConstArea& to) const override;
    bool canChangeLane(const lanelet::ConstLanelet& from, const lanelet::ConstLanelet& to) const override;
    bool isOneWay(const lanelet::ConstLanelet& lanelet) const override;
    SpeedLimitInformation speedLimit(const lanelet::ConstLanelet& lanelet) const override;
    SpeedLimitInformation speedLimit(const lanelet::ConstArea& area) const override;
    bool hasDynamicRules(const lanelet::ConstLanelet& lanelet) const override;
};

}  // namespace traffic_rules
}  // namespace lanelet

#endif // AMR_TRAFFIC_RULES_HPP