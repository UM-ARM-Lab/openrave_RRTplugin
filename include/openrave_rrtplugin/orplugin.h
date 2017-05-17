#ifndef ORPLUGIN_H
#define ORPLUGIN_H

#include "openrave_rrtplugin/include/rrtHelper.h"


#include <openrave/plugin.h>
//#include <boost/bind.hpp>

#include <atomic>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace OpenRAVE;
using namespace Eigen;

using namespace orPlugin;


class rrtPlugin : public ModuleBase
{
public:
    rrtPlugin(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {

        penv_ = penv;
        penv_->GetRobots(robots_);

        RegisterCommand("MyCommand",boost::bind(&rrtPlugin::MyCommand,this,_1,_2),
                        "This is an example command");
    }
    virtual ~rrtPlugin() {}

    bool MyCommand(std::ostream& sout, std::istream& sinput);

    void SetBoundary(std::vector<double> lowerBoundIn, std::vector<double> upperBoundIn);

    void SetWeight(std::vector<double> weightIn);

    void SetGeodesic(tConfigSet SE3Geodesic, std::vector<RobotBasePtr> robots);

    void SetStarSE3(tConfigSet SE3start);

    void SetGoalSE3(tConfigSet SE3goal);

    void SetStepSize(float stepSize);

    void SetSampleBias(float sampleBias);

    void SetSmoothFlag(int isSmooth);

    void SetBiRRTFlag(int isBiRRT);

    void RrtPlanning();

    void SmoothPath();

    void BiRRtPlanning();


protected:
    std::vector<RobotBasePtr> robots_;
    EnvironmentBasePtr penv_;
    ParameterSet inputParameter_;

    RrtPlanner rrtPlanner_;

    std::vector<RRTNodePtr> nodePath_;
    std::vector<tConfigSet> SE3Path_;
    std::vector<configSet> configPath_;

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
{ }





#endif
