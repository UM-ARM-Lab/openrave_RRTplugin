#ifndef RRTHELPER_H
#define RRTHELPER_H

#include <openrave/openrave.h>
//#include <openrave/plugin.h>

#include <iostream>
#include <stdio.h>
#include <vector>

//#include <boost/bind.hpp>

#include <arc_utilities/arc_helpers.hpp>

#define ERRORTHRESHOLD 0.002
#define GOALERROR 0.3
#define PI 3.141592653
#define STEPSIZE 0.3
#define SAMPLEBIAS 0.1
#define NUMITERATION 200
#define DOFCONFIG 6
#define STRETCHING_THRESHOLD 0

using namespace std;
using namespace OpenRAVE;
using namespace Eigen;

namespace orPlugin {


    typedef std::vector<double> configuration;
//    typedef Eigen::Affine3d tConfiguration;
    typedef std::vector<configuration> configSet;
    typedef std::pair<Eigen::Affine3d, Eigen::Affine3d> tConfigSet;

    typedef configSet* configSetPtr;
    typedef tConfigSet* tConfigSetPtr;

    configuration SE3toVconfig(Eigen::Affine3d tConfig);
    configSet SetSE3toVconfig(tConfigSet tConfigSetIn);

    Eigen::Affine3d VtoSE3config(configuration config);
    tConfigSet SetVtoSE3config(configSet config);

    class RRTNode;
    typedef RRTNode* RRTNodePtr;
    typedef std::vector<RRTNodePtr> tree;

    class RRTNode
    {

    public:
        RRTNode();
        RRTNode(configSet configSetIn);
        ~RRTNode();

//        bool SameNode(RRTNodePtr checkNode);
        void SetConfig(configSet config);
        configSet GetConfig();
        void AddStep(configSet s);

        void SetParent(RRTNodePtr parentNode);
        RRTNodePtr GetParent();

        // NEWLY DECLARED FUNCTION, TO BE IMPLEMENT
//        bool IsChild(RRTNodePtr nodePtr);
//        void SetChild(RRTNodePtr nodePtr);
//        std::vector<RRTNodePtr> GetChild();

    private:
        configSet configSet_;
        RRTNodePtr parent_;

        // NEWLY DECLARED VARIABLE
//        std::vector<RRTNodePtr> children_;
    };

    class NodeTree
    {
    public:
        NodeTree();
        NodeTree(RRTNodePtr initNode);
        ~NodeTree();

        RRTNodePtr GetRoot();

        void ResetTree();
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
        void InitGeodesic(configSet initConfig, std::vector<RobotBasePtr> robots);

        void SetStart(tConfigSet startSE3);
        void SetStart(configSet start);

        void SetGoal(tConfigSet goalSE3);
        void SetGoal(configSet goal);

        // To simplify the problem, I assume all grippers having the same boundary
        std::vector<double> lowerBound;
        std::vector<double> upperBound;
        std::vector<double> weight;
        // NEW IN PROJECT
        int numGrippers = 2;
        configSet geodesicConfig;
        std::vector<float> geodesic;

        float sampleBias = SAMPLEBIAS;
        float stepSize = STEPSIZE;
        int iteration = NUMITERATION;
        int isSmooth = 0;
        int isBiRRT = 0;

        configSet start_;
        configSet goal_;
        tConfigSet startSE3_;
        tConfigSet goalSE3_;



    };

    typedef NodeTree* NodeTreePtr;

    /////////////////////////// Helper function /////////////////////////////////

    float Distance(configuration A, configuration B);
    bool SameNode(RRTNodePtr baseNode, RRTNodePtr checkNode);

    OpenRAVE::Vector ConfigToRaveVec(configuration config);

    OpenRAVE::Vector GetObjectPos(OpenRAVE::RobotBasePtr robotPtr);

    configuration RaveVecToConfig(OpenRAVE::Vector raveVec);

    configuration VecToConfig(std::vector<double> vec);

    std::vector<double> ConfigToVec(configuration config);



    class RrtPlanner
    {
    public:

        void init(OpenRAVE::EnvironmentBasePtr env);

        void init(OpenRAVE::EnvironmentBasePtr env, ParameterSet parameterIn);

        ///////////////////////// Parameters input
        void SetParameters(ParameterSet parameterIn);

        ParameterSet GetParameters();

        ///////////////////////// RRT planner helper function, for single configuration

        void ResetPath();

        int RandNode(int size);

        configuration RandSample();

        configuration ScaleConfig(configuration v, float scale);

        void ConfigPrintHelp(configuration config);

        // if configuration violate joint limits
        bool OutBound(configuration a);
        bool SameDirection(configuration a, configuration b);

        configuration SumConfig(configuration A, configuration B);
        // A-B
        configuration SubtractConfig(configuration A, configuration B);

        // return unitstep vector, the step size is in #define, could be adjust later
        configuration UnitStep(configuration start, configuration goal, float stepSize);

//        float Distance(configuration A, configuration B);

        float WeightedDis(configuration A, configuration B);

        ////////////////////// Helper function for configuration SET (Pair)
        ///

//        bool SameNode(RRTNodePtr baseNode, RRTNodePtr checkNode);

        void SetConfigPrintHelp(configSet config);

        bool SetOutBound(configSet a);

        configSet SetScaleConfig(configSet v, float scale);

        configSet SetVecToConfig(std::vector<std::vector<double> > setVec);

        std::vector<std::vector<double> > SetConfigToVec(configSet setConfig);

        float SetWeightedDis(configSet A, configSet B);

        configSet SetSumConfig(configSet A, configSet B);

        configSet SetSubtractConfig(configSet A, configSet B);  // A-B

        // Nearest Node on the tree
        RRTNode* NearestNode(configSet config, NodeTreePtr treePtr);

        configSet SampleSet(int gripperSize);

        configSet SampleSetWithBase(configuration baseSample, int index);

        configSet SetUnitStep(configSet start, configSet goal, float stepSize);

        // Constraint violation check for geodesic constraint, as well as collision
        // for the ray connecting two gasped area true if violation happens
        // sampleConfig here is in object config space;


        bool ConstraintViolation(configSet sampleConfig);

        // Set collision check helper
        bool SetCollisionCheck();

        ///////////////////////////////////// RRT Planning //////////////////////////

        std::vector<RRTNode *> RRTPlanning(ParameterSet parameterIn);
        //tree* RRTPlanning(OpenRAVE::EnvironmentBasePtr env,
        //                 configuration goalConfig, float sampleBias, float stepSize, configuration weightIn);


//        std::vector<RRTNode*> SmoothPath(EnvironmentBasePtr env, std::vector<RRTNode *> &path,float stepSize, int iteration);
        std::vector<RRTNode*> SmoothPath();

        std::vector<RRTNode*> BiRRTPlanning(ParameterSet parameterIn);


    public:
        ParameterSet inputParameters_;
        OpenRAVE::EnvironmentBasePtr env_;
        std::vector<RobotBasePtr> robots_;

    protected:
        NodeTree* treePtr_;
        std::vector<RRTNodePtr> path_;

        NodeTree* treeA_;
        NodeTree* treeB_;

    };


}


#endif
