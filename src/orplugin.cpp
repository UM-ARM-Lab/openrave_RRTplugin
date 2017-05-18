//#include "rrtHelper.h"
#include "openrave_rrtplugin/orplugin.h"

//#include <boost/bind.hpp>
//#include <victor_hardware_interface/MotionCommand.h>
#include <arc_utilities/ros_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/eigen_helpers.hpp>

/*
#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <atomic>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
*/


using namespace OpenRAVE;


using namespace orPlugin;



bool rrtPlugin::MyCommand(std::ostream& sout, std::istream& sinput)
{
    std::string input;
    sinput >> input;
    sout << "output";
    return true;
}


void rrtPlugin::SetBoundary(std::vector<double> &lowerBoundIn, std::vector<double> &upperBoundIn)
{
    rrtPlanner_.inputParameters_.lowerBound = lowerBoundIn;
    rrtPlanner_.inputParameters_.upperBound = upperBoundIn;
}

void rrtPlugin::SetWeight(std::vector<double> &weightIn)
{
    rrtPlanner_.inputParameters_.weight = weightIn;
}


void rrtPlugin::SetGeodesic(tConfigSet &SE3Geodesic)
{
    rrtPlanner_.inputParameters_.geodesicConfig = SetSE3toVconfig(SE3Geodesic);
    rrtPlanner_.inputParameters_.InitGeodesic(rrtPlanner_.inputParameters_.geodesicConfig, robots_);

}

void rrtPlugin::SetStarSE3(tConfigSet &SE3start)
{
    rrtPlanner_.inputParameters_.startSE3_ = SE3start;
    rrtPlanner_.inputParameters_.start_ = SetSE3toVconfig(SE3start);
}

void rrtPlugin::SetGoalSE3(tConfigSet &SE3goal)
{
    rrtPlanner_.inputParameters_.goalSE3_ = SE3goal;
    rrtPlanner_.inputParameters_.goal_ = SetSE3toVconfig(SE3goal);
}

void rrtPlugin::SetStepSize(float stepSize)
{
    rrtPlanner_.inputParameters_.stepSize = stepSize;
}

void rrtPlugin::SetSampleBias(float sampleBias)
{
    rrtPlanner_.inputParameters_.sampleBias = sampleBias;
}

void rrtPlugin::SetNumIteration(int numIterationIn)
{
    rrtPlanner_.inputParameters_.iteration = numIterationIn;
}


void rrtPlugin::SetSmoothFlag(int isSmooth)
{
    rrtPlanner_.inputParameters_.isSmooth = isSmooth;
}

void rrtPlugin::SetBiRRTFlag(int isBiRRT)
{
    rrtPlanner_.inputParameters_.isBiRRT = isBiRRT;
}

void rrtPlugin::SetPath(std::vector<RRTNodePtr> pathIn)
{
    nodePath_ = pathIn;
    configPath_.clear();
    SE3Path_.clear();
    for (int i =0; i<pathIn.size(); i++)
    {
        configPath_.push_back(pathIn.at(i)->GetConfig());
        SE3Path_.push_back(SetVtoSE3config(configPath_.at(i)));
    }

}


void rrtPlugin::RrtPlanning()
{
    SetPath(rrtPlanner_.RRTPlanning(rrtPlanner_.inputParameters_));
}

void rrtPlugin::SmoothPath()
{
    SetPath(rrtPlanner_.SmoothPath());
}

void rrtPlugin::BiRRtPlanning()
{
    SetPath(rrtPlanner_.BiRRTPlanning(rrtPlanner_.inputParameters_));
}


