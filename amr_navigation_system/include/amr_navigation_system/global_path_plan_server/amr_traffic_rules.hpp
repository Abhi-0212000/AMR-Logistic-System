// Copyright 2025 Abhishek Nannuri
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file amr_traffic_rules.hpp
 * @brief Defines the AmrTrafficRules class for autonomous mobile robot navigation.
 *
 * This file contains the declaration of the AmrTrafficRules class, which extends
 * Lanelet2's TrafficRules to implement specific rules for autonomous mobile robots.
 */

#ifndef AMR_NAVIGATION_SYSTEM__GLOBAL_PATH_PLAN_SERVER__AMR_TRAFFIC_RULES_HPP_
#define AMR_NAVIGATION_SYSTEM__GLOBAL_PATH_PLAN_SERVER__AMR_TRAFFIC_RULES_HPP_

#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

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
  explicit AmrTrafficRules(Configuration config = Configuration());

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

#endif  // AMR_NAVIGATION_SYSTEM__GLOBAL_PATH_PLAN_SERVER__AMR_TRAFFIC_RULES_HPP_
