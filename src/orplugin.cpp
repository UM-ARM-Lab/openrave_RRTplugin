//#include "rrtHelper.h"
#include "openrave_rrtplugin/orplugin.h"

#include <boost/bind.hpp>
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


void rrtPlugin::SetBoundary(std::vector<double> lowerBoundIn, std::vector<double> upperBoundIn)
{
    inputParameter_.lowerBound = lowerBoundIn;
    inputParameter_.upperBound = upperBoundIn;
}

void rrtPlugin::SetWeight(std::vector<double> weightIn)
{
    inputParameter_.weight = weightIn;
}

/*
void rrtPlugin::SetGeodesic(tConfigSet SE3Geodesic, std::vector<RobotBasePtr> robots)
{
    inputParameter_.geodesicConfig = SetSE3toVconfig(SE3Geodesic);
    inputParameter_.InitGeodesic(inputParameter_.geodesicConfig, robots);

}
*/

void rrtPlugin::SetGeodesic(tConfigSet SE3Geodesic, std::vector<RobotBasePtr> robots)
{
//    inputParameter_.InitGeodesic(SetSE3toVconfig(SE3Geodesic), robots);
}

/*
void SetStarSE3(tConfigSet SE3start);

void SetGoalSE3(tConfigSet SE3goal);

void SetStepSize(float stepSize);

void SetSampleBias(float sampleBias);

void SetSmoothFlag(int isSmooth);

void SetBiRRTFlag(int isBiRRT);

void RrtPlanning();

void SmoothPath();

void BiRRtPlanning();

*/
