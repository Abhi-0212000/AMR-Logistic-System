/**
 * @file amr_traffic_rules.hpp
 * @brief Defines the AmrTrafficRules class for autonomous mobile robot navigation.
 *
 * This file contains the declaration of the AmrTrafficRules class, which extends
 * Lanelet2's TrafficRules to implement specific rules for autonomous mobile robots.
 */

#ifndef AMR_TRAFFIC_RULES_HPP
#define AMR_TRAFFIC_RULES_HPP

#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

namespace lanelet
{
namespace traffic_rules
{

/**
 * @class AmrTrafficRules
 * @brief Implements traffic rules for autonomous mobile robots.
 *
 * This class extends Lanelet2's TrafficRules to provide specific rule implementations
 * for autonomous mobile robots navigating in a mapped environment.
 */
class AmrTrafficRules : public TrafficRules
{
public:
  /**
     * @brief Construct a new AmrTrafficRules object
     */
  AmrTrafficRules();

  /**
     * @brief Check if the AMR can pass through a given lanelet
     * @param lanelet The lanelet to check
     * @return true if the AMR can pass, false otherwise
     */
  bool canPass(const lanelet::ConstLanelet & lanelet) const override;

  /**
     * @brief Check if the AMR can pass through a given area
     * @param area The area to check
     * @return true if the AMR can pass, false otherwise
     */
  bool canPass(const lanelet::ConstArea & area) const override;

  /**
     * @brief Check if the AMR can pass from one lanelet to another
     * @param from The starting lanelet
     * @param to The destination lanelet
     * @return true if the AMR can pass between the lanelets, false otherwise
     */
  bool canPass(const lanelet::ConstLanelet & from, const lanelet::ConstLanelet & to) const override;

  /**
     * @brief Check if the AMR can pass from a lanelet to an area
     * @param from The starting lanelet
     * @param to The destination area
     * @return true if the AMR can pass from the lanelet to the area, false otherwise
     */
  bool canPass(const lanelet::ConstLanelet & from, const lanelet::ConstArea & to) const override;

  /**
     * @brief Check if the AMR can pass from an area to a lanelet
     * @param from The starting area
     * @param to The destination lanelet
     * @return true if the AMR can pass from the area to the lanelet, false otherwise
     */
  bool canPass(const lanelet::ConstArea & from, const lanelet::ConstLanelet & to) const override;

  /**
     * @brief Check if the AMR can pass from one area to another
     * @param from The starting area
     * @param to The destination area
     * @return true if the AMR can pass between the areas, false otherwise
     */
  bool canPass(const lanelet::ConstArea & from, const lanelet::ConstArea & to) const override;

  /**
     * @brief Check if the AMR can change lane from one lanelet to another
     * @param from The current lanelet
     * @param to The target lanelet
     * @return true if the AMR can change lane, false otherwise
     */
  bool canChangeLane(
    const lanelet::ConstLanelet & from, const lanelet::ConstLanelet & to) const override;

  /**
     * @brief Check if a lanelet is one-way
     * @param lanelet The lanelet to check
     * @return true if the lanelet is one-way, false otherwise
     */
  bool isOneWay(const lanelet::ConstLanelet & lanelet) const override;

  /**
     * @brief Get the speed limit for a given lanelet
     * @param lanelet The lanelet to check
     * @return SpeedLimitInformation containing the speed limit
     */
  SpeedLimitInformation speedLimit(const lanelet::ConstLanelet & lanelet) const override;

  /**
     * @brief Get the speed limit for a given area
     * @param area The area to check
     * @return SpeedLimitInformation containing the speed limit
     */
  SpeedLimitInformation speedLimit(const lanelet::ConstArea & area) const override;

  /**
     * @brief Check if a lanelet has dynamic rules
     * @param lanelet The lanelet to check
     * @return true if the lanelet has dynamic rules, false otherwise
     */
  bool hasDynamicRules(const lanelet::ConstLanelet & lanelet) const override;
};

}  // namespace traffic_rules
}  // namespace lanelet

#endif  // AMR_TRAFFIC_RULES_HPP
