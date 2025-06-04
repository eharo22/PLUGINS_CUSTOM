/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "WindTests.hh"

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

#include <google/protobuf/message.h>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <gz/msgs/wind.pb.h>

#include <string>
#include <utility>
#include <vector>
#include <fstream>
#include <sstream>

#include <sdf/Root.hh>
#include <sdf/Error.hh>

#include <gz/common/Profiler.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/SdfEntityCreator.hh"

#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/LinearVelocitySeed.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Wind.hh"
#include "gz/sim/components/WindMode.hh"

#include "gz/sim/Link.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private WindTests data class.
class gz::sim::systems::WindTestsPrivate
{
  /// \brief Initialize the system.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  /// \param[in] _sdf Pointer to sdf::Element that contains configuration
  /// parameters for the system.
  public: void Load(EntityComponentManager &_ecm,
                    const std::shared_ptr<const sdf::Element> &_sdf);

  /// \brief Create subscribed topics and services.
  /// \param[in] _worldName Name of world entity.
  public: void SetupTransport(const std::string &_worldName);

  /// \brief Update the wind velocity component from CSV data.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void UpdateWindVelocity(const UpdateInfo &_info,
                                  EntityComponentManager &_ecm);

  /// \brief Calculate and apply forces on links affected by wind.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void ApplyWindForce(const UpdateInfo &_info,
                              EntityComponentManager &_ecm);

  /// \brief Load wind data from CSV file
  /// \return True if data was loaded successfully
  public: bool LoadWindData();

  /// \brief Process commands received from transport
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void ProcessCommandQueue(EntityComponentManager &_ecm);

  /// \brief Callback for servicing requests.
  /// \param[in] _msg msgs::Wind message.
  public: bool WindInfoService(msgs::Wind &_msg);

  /// \brief World entity to which this system is attached.
  public: Entity worldEntity;

  /// \brief Wind entity on which this sytem operates.
  public: Entity windEntity;

  /// \brief Path to CSV file containing wind data
  public: std::string csvFilePath;

  /// \brief Duration to apply each wind data point (seconds)
  public: double timePerDataPoint{1.0};

  /// \brief Vector to store wind speed data from CSV
  public: std::vector<double> windSpeeds;

  /// \brief Vector to store wind direction data from CSV
  public: std::vector<double> windDirections;

  /// \brief Current index in the wind data vectors
  public: size_t currentWindIndex{0};

  /// \brief Time when current wind data was applied
  public: std::chrono::steady_clock::time_point lastWindUpdateTime;

  /// \brief The scaling factor to approximate wind as force on a mass.
  public: double forceApproximationScalingFactor{1.0};

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Set during Load to true if the configuration for the plugin is
  /// valid.
  public: bool validConfig{false};

  /// \brief Mutex to protect wind data
  public: std::mutex windInfoMutex;

  /// \brief Current wind velocity and global enable/disable state.
  public: msgs::Wind currentWindInfo;
};

/////////////////////////////////////////////////
void WindTestsPrivate::Load(EntityComponentManager &_ecm,
                              const std::shared_ptr<const sdf::Element> &_sdf)
{
  this->windEntity = _ecm.EntityByComponents(components::Wind());

  // Get CSV file path from SDF
  if (!_sdf->HasElement("csv_file"))
  {
    gzerr << "No <csv_file> specified in SDF" << std::endl;
    return;
  }
  this->csvFilePath = _sdf->Get<std::string>("csv_file");

  // Get time per data point from SDF
  if (_sdf->HasElement("time_per_data_point"))
  {
    this->timePerDataPoint = _sdf->Get<double>("time_per_data_point");
    if (this->timePerDataPoint <= 0)
    {
      gzerr << "<time_per_data_point> must be greater than 0" << std::endl;
      return;
    }
  }

  // Get force approximation scaling factor
  if (_sdf->HasElement("force_approximation_scaling_factor"))
  {
    this->forceApproximationScalingFactor = 
        _sdf->Get<double>("force_approximation_scaling_factor");
    if (this->forceApproximationScalingFactor <= 0)
    {
      gzerr << "<force_approximation_scaling_factor> must be greater than 0" 
            << std::endl;
      return;
    }
  }

  // Load wind data from CSV
  if (!this->LoadWindData())
  {
    gzerr << "Failed to load wind data from CSV file" << std::endl;
    return;
  }

  this->validConfig = true;
  this->lastWindUpdateTime = std::chrono::steady_clock::now();
}

//////////////////////////////////////////////////
bool WindTestsPrivate::LoadWindData()
{
  std::ifstream file(this->csvFilePath);
  if (!file.is_open())
  {
    gzerr << "Failed to open CSV file: " << this->csvFilePath << std::endl;
    return false;
  }

  std::string line;
  while (std::getline(file, line))
  {
    std::istringstream iss(line);
    std::string speedStr, directionStr;
    
    if (!std::getline(iss, speedStr, ',') || 
        !std::getline(iss, directionStr))
    {
      gzwarn << "Skipping malformed line in CSV: " << line << std::endl;
      continue;
    }

    try {
      double speed = std::stod(speedStr);
      double direction = std::stod(directionStr);
      
      this->windSpeeds.push_back(speed);
      this->windDirections.push_back(direction);
    }
    catch (const std::exception& e) {
      gzwarn << "Skipping line with invalid data: " << line 
             << " Error: " << e.what() << std::endl;
    }
  }

  if (this->windSpeeds.empty())
  {
    gzerr << "No valid wind data found in CSV file" << std::endl;
    return false;
  }

  gzmsg << "Loaded " << this->windSpeeds.size() 
        << " wind data points from CSV file" << std::endl;
  return true;
}

//////////////////////////////////////////////////
void WindTestsPrivate::SetupTransport(const std::string &_worldName)
{
  auto validWorldName = transport::TopicUtils::AsValidTopic(_worldName);
  if (validWorldName.empty())
  {
    gzerr << "Failed to setup transport, invalid world name [" << _worldName
           << "]" << std::endl;
    return;
  }

  // Wind info service
  this->node.Advertise("/world/" + validWorldName + "/wind_info",
                       &WindTestsPrivate::WindInfoService, this);
}

//////////////////////////////////////////////////
void WindTestsPrivate::UpdateWindVelocity(const UpdateInfo &_info,
                                            EntityComponentManager &_ecm)
{
  GZ_PROFILE("WindTestsPrivate::UpdateWindVelocity");
  
  // Check if it's time to update the wind data
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
      now - this->lastWindUpdateTime).count();

  if (elapsed >= this->timePerDataPoint)
  {
    this->currentWindIndex = (this->currentWindIndex + 1) % this->windSpeeds.size();
    this->lastWindUpdateTime = now;
  }

  // Get current wind data
  double speed = this->windSpeeds[this->currentWindIndex];
  double direction = this->windDirections[this->currentWindIndex];

  // Convert direction (degrees) to radians and calculate components
  double directionRad = GZ_DTOR(direction);
  math::Vector3d windVel(
      speed * std::cos(directionRad),
      speed * std::sin(directionRad),
      0.0);

  // Update component
  auto windLinVel =
      _ecm.Component<components::WorldLinearVelocity>(this->windEntity);
  if (windLinVel)
  {
    windLinVel->Data() = windVel;
  }

  // Update current wind info for services
  std::lock_guard<std::mutex> lock(this->windInfoMutex);
  msgs::Set(this->currentWindInfo.mutable_linear_velocity(), windVel);
}

//////////////////////////////////////////////////
void WindTestsPrivate::ApplyWindForce(const UpdateInfo &,
                                        EntityComponentManager &_ecm)
{
  GZ_PROFILE("WindTestsPrivate::ApplyWindForce");
  auto windVel =
      _ecm.Component<components::WorldLinearVelocity>(this->windEntity);
  if (!windVel)
    return;

  Link link;

  _ecm.Each<components::Link,
            components::Inertial,
            components::WindMode,
            components::WorldPose,
            components::WorldLinearVelocity>(
      [&](const Entity &_entity,
          components::Link *,
          components::Inertial *_inertial,
          components::WindMode *_windMode,
          components::WorldPose *_linkPose,
          components::WorldLinearVelocity *_linkVel) -> bool
      {
        // Skip links for which the wind is disabled
        if (!_windMode->Data())
        {
          return true;
        }

        link.ResetEntity(_entity);

        const math::Vector3d windForce =
            _inertial->Data().MassMatrix().Mass() *
            this->forceApproximationScalingFactor * 
            (windVel->Data() - _linkVel->Data());

        // Apply force at center of mass
        link.AddWorldForce(_ecm, windForce);

        return true;
      });
}

//////////////////////////////////////////////////
void WindTestsPrivate::ProcessCommandQueue(EntityComponentManager &_ecm)
{
  // Not used in this version but kept for interface compatibility
}

//////////////////////////////////////////////////
bool WindTestsPrivate::WindInfoService(msgs::Wind &_msg)
{
  std::lock_guard<std::mutex> lock(this->windInfoMutex);
  _msg.CopyFrom(this->currentWindInfo);
  return true;
}

//////////////////////////////////////////////////
WindTests::WindTests() : System(),
    dataPtr(std::make_unique<WindTestsPrivate>())
{
}

//////////////////////////////////////////////////
WindTests::~WindTests() = default;

//////////////////////////////////////////////////
void WindTests::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{
  this->dataPtr->worldEntity = _entity;
  this->dataPtr->Load(_ecm, _sdf);

  if (this->dataPtr->validConfig)
  {
    auto worldName =
        _ecm.Component<components::Name>(this->dataPtr->worldEntity);
    if (worldName)
    {
      this->dataPtr->SetupTransport(worldName->Data());
    }

    // By default, the wind system is enabled.
    {
      std::lock_guard lock(this->dataPtr->windInfoMutex);
      this->dataPtr->currentWindInfo.set_enable_wind(true);
    }
  }
}

//////////////////////////////////////////////////
void WindTests::PreUpdate(const UpdateInfo &_info,
                            EntityComponentManager &_ecm)
{
  GZ_PROFILE("WindTests::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  if (!this->dataPtr->validConfig)
    return;

  _ecm.EachNew<components::Link, components::WindMode>(
      [&](const Entity &_entity, components::Link *,
          components::WindMode *_windMode) -> bool
  {
    if (_windMode->Data())
    {
      Link link(_entity);
      link.EnableVelocityChecks(_ecm, true);
    }
    return true;
  });

  if (_info.paused)
    return;

  this->dataPtr->UpdateWindVelocity(_info, _ecm);
  this->dataPtr->ApplyWindForce(_info, _ecm);
}

GZ_ADD_PLUGIN(WindTests, System,
  WindTests::ISystemConfigure,
  WindTests::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(WindTests, "gz::sim::systems::WindTests")