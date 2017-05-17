#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include "rrtHelper.h"

using namespace std;
using namespace OpenRAVE;

class MyNewModule : public ModuleBase
{
public:
    MyNewModule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {

        penv_=penv;

        RegisterCommand("MyCommand",boost::bind(&MyNewModule::MyCommand,this,_1,_2),
                        "This is an example command");

        RegisterCommand("goal",boost::bind(&MyNewModule::SetGoal,this,_1,_2),
                        "goal %f %f %f %f %f %f %f;");

        RegisterCommand("bias",boost::bind(&MyNewModule::SetBias,this,_1,_2),
                        "bias %f;");

        RegisterCommand("step",boost::bind(&MyNewModule::SetStep,this,_1,_2),
                        "step %f;");

        RegisterCommand("isBiRRT",boost::bind(&MyNewModule::SetBiRRT,this,_1,_2),
                        "isBiRRT %d;");

        RegisterCommand("SetWeight",boost::bind(&MyNewModule::SetWeight,this,_1,_2),
                        "SetWeight ;");

        RegisterCommand("SetUpperBound",boost::bind(&MyNewModule::SetUpperBound,this,_1,_2),
                        "SetUpperBound %f %f %f %f %f %f %f;");

        RegisterCommand("SetLowerBound",boost::bind(&MyNewModule::SetLowerBound,this,_1,_2),
                        "SetLowerBound %f %f %f %f %f %f %f;");

        RegisterCommand("RRT",boost::bind(&MyNewModule::RRT,this,_1,_2),
                        "Run RRT");

        RegisterCommand("BiRRT",boost::bind(&MyNewModule::BiRRT,this,_1,_2),
                        "Run BiRRT");

        RegisterCommand("Smooth",boost::bind(&MyNewModule::Smooth,this,_1,_2),
                        "Smooth %d;");

        RegisterCommand("getRRTpath",boost::bind(&MyNewModule::GetRRTPath,this,_1,_2),
                        "Path %d;");




    }
    virtual ~MyNewModule() {}

    EnvironmentBasePtr penv_;
//    ParameterSet plannerSetup;

    configSet goalConfig;
    float goalBias = SAMPLEBIAS;
    float stepSize = STEPSIZE;
    int smooth = 0;
    int isBiRRT = 0;
    std::vector<RRTNode*> path;

    
    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
/*
        vector<OpenRAVE::RobotBasePtr> robots;
        penv_->GetRobots(robots);
        std::vector<KinBody::LinkPtr> LPtr = (robots.at(0)->GetLinks());
        Transform trans = (LPtr.at(6))->GetTransform();
        OpenRAVE::Vector pos = trans.trans;
        double x = pos.x;
*/


        plannerSetup.weight.clear();
        plannerSetup.lowerBound.clear();
        plannerSetup.upperBound.clear();


        plannerSetup.weight.push_back(1.0);
        plannerSetup.weight.push_back(1.0);
        plannerSetup.weight.push_back(1.0);

        plannerSetup.weight.push_back(0.0);
        plannerSetup.weight.push_back(0.0);
        plannerSetup.weight.push_back(0.0);

        plannerSetup.lowerBound.push_back(-2.0);
        plannerSetup.lowerBound.push_back(-2.0);
        plannerSetup.lowerBound.push_back(0.0);

        plannerSetup.lowerBound.push_back(-PI);
        plannerSetup.lowerBound.push_back(-PI);   // 10000.0
        plannerSetup.lowerBound.push_back(-PI);

        plannerSetup.upperBound.push_back(2.0);
        plannerSetup.upperBound.push_back(2.0);
        plannerSetup.upperBound.push_back(2.5);

        plannerSetup.upperBound.push_back(PI);
        plannerSetup.upperBound.push_back(PI);  // 10000
        plannerSetup.upperBound.push_back(PI);


        plannerSetup.numGrippers=2;

        configuration ini1;
        configuration ini2;
        ini1.push_back(0.5);
        ini1.push_back(0.0);
        ini1.push_back(2.0);
        ini1.push_back(0.5);
        ini1.push_back(0.0);
        ini1.push_back(0.0);

        ini2.push_back(0.5);
        ini2.push_back(0.2);
        ini2.push_back(2.0);
        ini2.push_back(0.0);
        ini2.push_back(0.0);
        ini2.push_back(0.0);


        configSet setConfig;
        setConfig.push_back(ini1);
        setConfig.push_back(ini2);
        plannerSetup.InitGeodesic(setConfig);

//        configSet A = SampleSet(plannerSetup.numGrippers);

//        configSet B;
//        B.push_back(ini1);
//        B.push_back(ini2);
//        SetOutBound(A);
//        std::vector<std::vector<double> > Avec = SetConfigToVec(A);
//        SetVecToConfig(Avec);
//        SetWeightedDis(A,B);
//        OpenRAVE::Vector raveA = ConfigToRaveVec(A.at(0));

        cout << "Comment sent to robot before entering function-------" << endl;

        /*
        for (int i =0; i<robots.size();i++)
        {
            robots.at(i)->SetActiveDOFValues(ConfigToVec(ScaleConfig(A.at(i),(1+i)*2)));
            ConfigPrintHelp(ScaleConfig(A.at(i),(1+i)*2));
        }
        */
//        ConstraintViolation(SetSumConfig(A,B), robots, penv_);
//        SetCollisionCheck(robots,penv_);

//        path = RRTPlanning(penv_,B,0.11,0.4);
        path = RRTPlanning(penv_,goalConfig,goalBias,stepSize);

        return true;
    }

    bool SetGoal(std::ostream& sout, std::istream& sinput)
    {
        /*
        std::string input;
        sinput >> input;
        int g_ind;
        g_ind = std::atoi(input.c_str());
        */

        configuration goal;
        for (int i=0; i<DOFCONFIG; i++)
        {
            std::string input;
            sinput >> input;
            goal.push_back(std::strtof(input.c_str(),0));
        }
        goalConfig.push_back(goal);
        sout << "Get Goal Configuration!";
        return true;
    }

    bool SetBias(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        goalBias = std::strtof(input.c_str(),0);
        sout << "Get Bias !";

        return true;
    }

    bool SetStep(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        stepSize = std::strtof(input.c_str(),0);
        sout << "Get Step !";

        return true;
    }

    bool SetBiRRT(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        isBiRRT = std::atoi(input.c_str());;
        sout << "Get BiRRT Flag !";

        return true;
    }

    bool SetWeight(std::ostream& sout, std::istream& sinput)
    {
        plannerSetup.weight.clear();

        plannerSetup.weight.push_back(1.0);
        plannerSetup.weight.push_back(1.0);
        plannerSetup.weight.push_back(1.0);

        plannerSetup.weight.push_back(0.0);
        plannerSetup.weight.push_back(0.0);
        plannerSetup.weight.push_back(0.0);

        return true;
    }

    bool SetUpperBound(std::ostream& sout, std::istream& sinput)
    {
        plannerSetup.upperBound.clear();
        for (int i=0; i<DOFCONFIG; i++)
        {
            std::string input;
            sinput >> input;
            plannerSetup.upperBound.push_back(std::strtof(input.c_str(),0));
        }
        sout << "Get UpperBound!";
        return true;

        /*
        plannerSetup.lowerBound.clear();
        plannerSetup.upperBound.clear();

        plannerSetup.lowerBound.push_back(-7.0);
        plannerSetup.lowerBound.push_back(-7.0);
        plannerSetup.lowerBound.push_back(0.0);

        plannerSetup.lowerBound.push_back(-PI);
        plannerSetup.lowerBound.push_back(-PI);   // 10000.0
        plannerSetup.lowerBound.push_back(-PI);

        plannerSetup.upperBound.push_back(7.0);
        plannerSetup.upperBound.push_back(7.0);
        plannerSetup.upperBound.push_back(7);

        plannerSetup.upperBound.push_back(PI);
        plannerSetup.upperBound.push_back(PI);  // 10000
        plannerSetup.upperBound.push_back(PI);

        return true;
        */
    }

    bool SetLowerBound(std::ostream& sout, std::istream& sinput)
    {
        plannerSetup.lowerBound.clear();
        for (int i=0; i<DOFCONFIG; i++)
        {
            std::string input;
            sinput >> input;
            plannerSetup.lowerBound.push_back(std::strtof(input.c_str(),0));
        }
        sout << "Get LowerBound!";
        return true;
    }


    bool RRT(std::ostream& sout, std::istream& sinput)
    {
//        RRTPlanning(penv_, goalConfig, goalBias, stepSize);
        SetConfigPrintHelp(goalConfig);

        plannerSetup.numGrippers = goalConfig.size();

        /*
        configuration ini1;
        configuration ini2;
        ini1.push_back(0.5);
        ini1.push_back(0.0);
        ini1.push_back(2.0);
        ini1.push_back(0.5);
        ini1.push_back(0.0);
        ini1.push_back(0.0);

        ini2.push_back(0.5);
        ini2.push_back(0.2);
        ini2.push_back(2.0);
        ini2.push_back(0.0);
        ini2.push_back(0.0);
        ini2.push_back(0.0);

        configSet setConfig;
        setConfig.push_back(ini1);
        setConfig.push_back(ini2);
        */

        vector<OpenRAVE::RobotBasePtr> robots;
        penv_->GetRobots(robots);
        std::vector<std::vector<double> > startSet;

        for (int i =0; i<plannerSetup.numGrippers; i++)
        {
            std::vector<double> startVec;
            robots.at(i)->GetActiveDOFValues(startVec);
            startSet.push_back(startVec);
        }

        plannerSetup.InitGeodesic(SetVecToConfig(startSet));

        path = RRTPlanning(penv_, goalConfig, goalBias, stepSize);

        // print geodesic
        cout << "************ Geodesic Alongside ******" <<endl;
        for (int i=0; i<path.size();i++)
        {
            configSet config = path.at(i)->GetConfig();
            cout<<WeightedDis(config.at(0),config.at(1))<<endl;
        }


        cout << "The line after Path = " << endl;
        sout << "RRT Planning";
        return true;
    }

    bool BiRRT(std::ostream& sout, std::istream& sinput)
    {
        SetConfigPrintHelp(goalConfig);

        plannerSetup.numGrippers = goalConfig.size();

        vector<OpenRAVE::RobotBasePtr> robots;
        penv_->GetRobots(robots);
        std::vector<std::vector<double> > startSet;

        for (int i =0; i<plannerSetup.numGrippers; i++)
        {
            std::vector<double> startVec;
            robots.at(i)->GetActiveDOFValues(startVec);
            startSet.push_back(startVec);
        }

        plannerSetup.InitGeodesic(SetVecToConfig(startSet));

        path = RRTPlanning(penv_, goalConfig, goalBias, stepSize);

        // print geodesic
        cout << "************ Geodesic Alongside ******" <<endl;
        for (int i=0; i<path.size();i++)
        {
            configSet config = path.at(i)->GetConfig();
            cout<<WeightedDis(config.at(0),config.at(1))<<endl;
        }

        sout << "BiRRT Planning";
        return true;
    }


    bool Smooth(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        smooth = std::atoi(input.c_str());;
        sout << "Get BiRRT Flag !";

        path = SmoothPath(penv_, path,stepSize,smooth);

        sout << "Path Smooth!" ;

        return true;
    }


    bool GetRRTPath(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        int g_ind;
        g_ind = std::atoi(input.c_str());

        configSet nodeConfig;
        configuration config;
        float value;
        for (int i=0; i<path.size(); i++)
        {
            nodeConfig = (path.at(i))->GetConfig();
            config = nodeConfig.at(g_ind);
            for(int j=0; j<DOFCONFIG ; j++)
            {
                value = config.at(j);
                sout<< value;
                if (j==(config.size()-1)) { sout<<endl; }
                else { sout<<","; }
            }
        }
        return true;
    }



};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "mynewmodule" ) {
        return InterfaceBasePtr(new MyNewModule(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("MyNewModule");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

