#include <openrave_rrtplugin//orplugin.h>

#include <victor_hardware_interface/MotionCommand.h>
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


namespace orPlugin {



    bool rrtPlugin::MyCommand(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        sout << "output";
        return true;
    }



}
