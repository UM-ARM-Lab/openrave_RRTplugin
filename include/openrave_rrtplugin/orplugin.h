#ifndef ORPLUGIN_H
#define ORPLUGIN_H

#include "rrtHelper.h"

#include <openrave/plugin.h>
#include <boost/bind.hpp>

#include <atomic>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace OpenRAVE;
using namespace Eigen;

namespace orPlugin {


    class rrtPlugin : public ModuleBase
    {
    public:
        rrtPlugin(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
            RegisterCommand("MyCommand",boost::bind(&rrtPlugin::MyCommand,this,_1,_2),
                            "This is an example command");
        }
        virtual ~rrtPlugin() {}


        bool MyCommand(std::ostream& sout, std::istream& sinput);


    protected:
        std::vector<RobotBasePtr> robots_;

    };


    // called to create a new plugin
    InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
    {
        if( type == PT_Module && interfacename == "rrtplugin" ) {
            return InterfaceBasePtr(new rrtPlugin(penv,sinput));
        }

        return InterfaceBasePtr();
    }

    // called to query available plugins
    void GetPluginAttributesValidated(PLUGININFO& info)
    {
        info.interfacenames[PT_Module].push_back("rrtPlugin");
    }

    // called before plugin is terminated
    OPENRAVE_PLUGIN_API void DestroyPlugin()
    {
    }


}


#endif
