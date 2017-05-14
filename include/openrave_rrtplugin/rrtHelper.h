#ifndef RRTHELPER_H
#define RRTHELPER_H

#include <openrave/openrave.h>
#include <openrave/plugin.h>

#include <iostream>
#include <stdio.h>
#include <vector>

#include <boost/bind.hpp>
#include <time.h>
#include <stdlib.h>

#include <arc_utilities/arc_helpers.hpp>

#define ERRORTHRESHOLD 0.002
#define GOALERROR 0.3
#define PI 3.141592653
#define STEPSIZE 0.2
#define SAMPLEBIAS 0.15
#define PERIOD 100
#define DOFCONFIG 6
#define STRETCHING_THRESHOLD 0

using namespace std;
using namespace OpenRAVE;
using namespace Eigen;

namespace orPlugin {


    typedef std::vector<double> configuration;
//    typedef Eigen::Affine3d tConfiguration;
    typedef std::pair<configuration, configuration> configSet;
    typedef std::pair<Eigen::Affine3d, Eigen::Affine3d> tConfigSet;

    typedef configSet* configSetPtr;
    typedef tConfigSet* tConfigSetPtr;

    configuration TtoVconfig(Eigen::Affine3d tConfig);
    configSet SetTtoVconfig(tConfigSet tConfigSetIn);

    class RRTNode;
    typedef RRTNode* RRTNodePtr;
    typedef std::vector<RRTNodePtr> tree;

    class RRTNode
    {
    public:
        RRTNode();
        RRTNode(configSet configSetIn);
        ~RRTNode();

        bool SameNode(RRTNodePtr checkNode);
        void SetConfig(configSet config);
        configSet GetConfig();
        void AddStep(configSet s);

        void SetParent(RRTNodePtr);
        RRTNodePtr GetParent();

        // NEWLY DECLARED FUNCTION, TO BE IMPLEMENT
        bool IsChild(RRTNodePtr);
        void SetChild(RRTNodePtr);
        std::vector<RRTNodePtr> GetChild();

    private:
        configSet configSet_;
        RRTNodePtr parent_;

        // NEWLY DECLARED VARIABLE
        std::vector<RRTNodePtr> children_;
    };

    class NodeTree
    {
    public:
        NodeTree();
        NodeTree(RRTNodePtr initNode);
        ~NodeTree();

        RRTNodePtr GetRoot();

        bool InTree(RRTNodePtr testNode);
        bool Add(RRTNodePtr growNode);
        bool Remove(RRTNodePtr remNode);
        RRTNodePtr GetNode(int index);
        int TreeSize();
        int GetIndex(RRTNodePtr findNode);
        bool IsRoot(RRTNodePtr checkNode);

        std::vector<RRTNodePtr> GetPath(RRTNodePtr goal);

    private:
        tree tree_;
        RRTNodePtr rootNode_;
    };

    typedef NodeTree* NodeTreePtr;

    class ParameterSet
    {
    public:
        ParameterSet();
    //    ~ParameterSet();

        // Geodesic of (i,j) gripper stored in i*n+j; n is num of grippers
        void InitGeodesic(configSet initConfig);

        // To simplify the problem, I assume all grippers having the same boundary
        std::vector<double> lowerBound;
        std::vector<double> upperBound;
        std::vector<double> weight;
        // NEW IN PROJECT
        int numGrippers = 2;
        configSet geodesicConfig;
        std::vector<float> geodesic;

    };

    class RrtPlanner
    {
    public:

        ///////////////////////// Parameters input
        void SetParameters(ParameterSet parameterIn);

        ParameterSet GetParameters();

        ///////////////////////// RRT planner helper function, for single configuration

        int RandNode(int size);

        configuration RandSample();

        configuration ScaleConfig(configuration v, float scale);

        void ConfigPrintHelp(configuration config);

        configuration VecToConfig(std::vector<double> vec);

        std::vector<double> ConfigToVec(configuration config);
        // if configuration violate joint limits
        bool OutBound(configuration a);
        bool SameDirection(configuration a, configuration b);

        configuration SumConfig(configuration A, configuration B);
        // A-B
        configuration SubtractConfig(configuration A, configuration B);

        float Distance(configuration A, configuration B);
        float WeightedDis(configuration A, configuration B);

        // return unitstep vector, the step size is in #define, could be adjust later
        configuration UnitStep(configuration start, configuration goal, float stepSize);


        ////////////////////// Helper function for configuration SET (Pair)

        void SetConfigPrintHelp(configSet config);

        bool SetOutBound(configSet a);

        configSet SetScaleConfig(configSet v, float scale);

        configSet SetVecToConfig(std::vector<std::vector<double> > setVec);

        std::vector<std::vector<double> > SetConfigToVec(configSet setConfig);

        float SetWeightedDis(configSet A, configSet B);

        configSet SetSumConfig(configSet A, configSet B);

        configSet SetSubtractConfig(configSet A, configSet B);  // A-B

        // Nearest Node on the tree
        RRTNode* NearestNode(NodeTree* treeA, configSet config);

        configSet SampleSet(int gripperSize);

        configSet SampleSetWithBase(configuration baseSample, int index);

        configSet SetUnitStep(configSet start, configSet goal, float stepSize);

        // Constraint violation check for geodesic constraint, as well as collision
        // for the ray connecting two gasped area true if violation happens
        // sampleConfig here is in object config space;

        configuration RaveVecToConfig(OpenRAVE::Vector raveVec);

        OpenRAVE::Vector ConfigToRaveVec(configuration config);

        OpenRAVE::Vector GetObjectPos(OpenRAVE::RobotBasePtr robotPtr);

        bool ConstraintViolation(configSet sampleConfig, vector<RobotBasePtr> robots, EnvironmentBasePtr env);

        // Set collision check helper
        bool SetCollisionCheck(vector<RobotBasePtr> robots, EnvironmentBasePtr env);

        ///////////////////////////////////// RRT Planning //////////////////////////

        std::vector<RRTNode *> RRTPlanning(OpenRAVE::EnvironmentBasePtr env,
                         configSet goalConfig, float sampleBias, float stepSize);
        //tree* RRTPlanning(OpenRAVE::EnvironmentBasePtr env,
        //                 configuration goalConfig, float sampleBias, float stepSize, configuration weightIn);


        std::vector<RRTNode*> SmoothPath(EnvironmentBasePtr env, std::vector<RRTNode *> &path,float stepSize, int iteration);


        std::vector<RRTNode*> BiRRTPlanning(OpenRAVE::EnvironmentBasePtr env,
            configSet goalConfig, float sampleBias, float stepSize);







    protected:
        ParameterSet inputParameters_;







    };


}


#endif
