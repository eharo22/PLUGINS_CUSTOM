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

#include "TableVerticalForce.hh"
#include <iostream>
#include <gz/sim/components/Link.hh>
#include <gz/sim/Link.hh>
#include <gz/math/Vector3.hh>

using namespace gz;
using namespace sim;
using namespace systems;

void TableVerticalForce::Configure(const Entity &_entity,
                                       const std::shared_ptr<const sdf::Element> &_sdf,
                                       EntityComponentManager &_ecm,
                                       EventManager &)
{
  this->forceStepTable.clear();

  sdf::ElementPtr stepForceElem = const_cast<sdf::Element*>(_sdf.get())->GetElement("step_force");
  while (stepForceElem)
  {
    sdf::ElementPtr forceElem = stepForceElem->GetElement("force");
    sdf::ElementPtr stepsElem = stepForceElem->GetElement("steps");
    if (forceElem && stepsElem)
    {
      double force = forceElem->Get<double>();
      int steps = stepsElem->Get<int>();
      this->forceStepTable.push_back(std::make_pair(force, steps));
    }
    stepForceElem = stepForceElem->GetNextElement("step_force");
  }

  if (this->forceStepTable.empty())
  {
    this->forceStepTable.push_back(std::make_pair(1.0, 100)); // Default
    gzmsg << "No step_force values specified. Using default force of 1.0 N for 100 steps." << std::endl;
  }
  else
  {
    gzmsg << "Loaded " << this->forceStepTable.size() << " force/step pairs." << std::endl;
  }

  this->currentIndex = 0;
  if (!this->forceStepTable.empty())
  {
    this->currentForceMagnitude = this->forceStepTable[0].first;
    this->stepsRemaining = this->forceStepTable[0].second;
  }
}

void TableVerticalForce::PreUpdate(const UpdateInfo &_info,
                                       EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (this->forceStepTable.empty())
    return;

  if (this->stepsRemaining > 0)
  {
    this->stepsRemaining--;
  }
  else
  {
    this->currentIndex++;
    if (this->currentIndex >= this->forceStepTable.size())
    {
      this->currentIndex = 0; // Loop the table
    }
    this->currentForceMagnitude = this->forceStepTable[this->currentIndex].first;
    this->stepsRemaining = this->forceStepTable[this->currentIndex].second;
  }

  math::Vector3d verticalForce(0, 0, this->currentForceMagnitude);

  _ecm.Each<components::Link>(
      [&](const long unsigned int &entityId,
          components::Link *linkComponent)
      {
        if (linkComponent)
        {
          Entity entity(entityId);
          Link link(entity);
          link.AddWorldForce(_ecm, verticalForce);
        }
        return true;
      });
}