#include "MyPlugin.hh"
#include <gz/plugin/Register.hh>
#include <iostream>

using namespace my_namespace;

void MyPlugin::Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &,
                         gz::sim::EntityComponentManager &,
                         gz::sim::EventManager &)
{
  std::cout << "Plugin configured for entity " << _entity << std::endl;
}

// FIXED: use full namespace-qualified name
GZ_ADD_PLUGIN(my_namespace::MyPlugin, gz::sim::System, gz::sim::ISystemConfigure)
