#ifndef SS_GAZEBO_LINK_EFFORT_PLUGIN_HH
#define SS_GAZEBO_LINK_EFFORT_PLUGIN_HH

// #include <boost/shared_ptr.hpp>
// #include <sdf/sdf.hh>
#include <gazebo/common/Plugin.hh>
#include <memory>

#include "ss_algorithm/API/RobotSimCppGeneral.h"

namespace gazebo_plugins
{
class LinkEffortPluginPrivate;

class LinkEffortPlugin : public gazebo::ModelPlugin
{
public:
    LinkEffortPlugin();
    virtual ~LinkEffortPlugin();
    virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
    std::unique_ptr<LinkEffortPluginPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif