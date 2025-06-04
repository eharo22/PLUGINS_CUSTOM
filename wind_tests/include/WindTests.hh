

#ifndef GZ_SIM_SYSTEMS_WINDTESTS_HH_
#define GZ_SIM_SYSTEMS_WINDTESTS_HH_

#include <memory>

#include "gz/sim/System.hh"

namespace gz
{
  namespace sim
  {
    // Forward declarations
    class EntityComponentManager;
    class EventManager;

    namespace systems
    {
      // Forward declarations
      class WindTestsPrivate; // Changed from WindEffectsPrivate

      class GZ_SIM_VISIBLE WindTests final :
        public System,
        public ISystemConfigure,
        public ISystemPreUpdate
      {
        /// \brief Constructor
        public: WindTests(); // Changed from WindEffects

        /// \brief Destructor
        public: ~WindTests() override; // Changed from WindEffects

        // Documentation inherited
        public: void Configure(const Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               EntityComponentManager &_ecm,
                               EventManager &_eventMgr) override;

        // Documentation inherited
        public: void PreUpdate(const UpdateInfo &_info,
                                EntityComponentManager &_ecm) override;

        /// \brief Private data pointer
        private: std::unique_ptr<WindTestsPrivate> dataPtr; // Changed from WindEffectsPrivate
      };
    }
  }
}

#endif