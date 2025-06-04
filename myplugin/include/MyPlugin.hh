#ifndef MY_PLUGIN_HH
#define MY_PLUGIN_HH

#include <gz/sim/System.hh>

namespace my_namespace
{
  class MyPlugin : public gz::sim::System,
                   public gz::sim::ISystemConfigure
  {
  public:
    void Configure(const gz::sim::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   gz::sim::EntityComponentManager &ecm,
                   gz::sim::EventManager &eventMgr) override;
  };
}

#endif
