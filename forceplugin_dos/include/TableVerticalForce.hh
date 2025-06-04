/*
 * Copyright (C) 2025 Your Name
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef GZ_SIM_SYSTEMS_TABLE_VERTICAL_FORCE_HH_
#define GZ_SIM_SYSTEMS_TABLE_VERTICAL_FORCE_HH_

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Entity.hh>
#include <gz/plugin/Register.hh>
#include <sdf/Element.hh>
#include <memory>
#include <vector>
#include <utility> // For std::pair

namespace gz::sim::systems
{
  /// \brief A system plugin that applies a sequence of vertical forces for specified durations.
  class TableVerticalForce : public System,
                             public ISystemConfigure,
                             public ISystemPreUpdate
  {
  public:
    /// \brief Constructor.
    TableVerticalForce() = default;

    /// \brief Called after the plugin is loaded.
    /// \param[in] _entity The world entity.
    /// \param[in] _sdf The SDF element describing the plugin.
    /// \param[in] _ecm The EntityComponentManager.
    /// \param[in] _eventMgr The EventManager.
    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override;

    /// \brief Called during each simulation update.
    /// \param[in] _info Update information.
    /// \param[in] _ecm The EntityComponentManager.
    void PreUpdate(const UpdateInfo &_info,
                   EntityComponentManager &_ecm) override;

  private:
    /// \brief Table of (force magnitude, number of steps) pairs.
    std::vector<std::pair<double, int>> forceStepTable;

    /// \brief The current force magnitude being applied.
    double currentForceMagnitude = 0.0;

    /// \brief The number of steps remaining for the current force.
    int stepsRemaining = 0;

    /// \brief The index of the current force in the table.
    size_t currentIndex = 0;
  };
} // namespace gz::sim::systems

// Register this plugin with Gazebo
GZ_ADD_PLUGIN(gz::sim::systems::TableVerticalForce,
              gz::sim::System,
              gz::sim::systems::TableVerticalForce::ISystemConfigure,
              gz::sim::systems::TableVerticalForce::ISystemPreUpdate)

#endif // GZ_SIM_SYSTEMS_TABLE_VERTICAL_FORCE_HH_