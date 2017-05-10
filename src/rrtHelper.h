#ifndef RRTHELPER_H
#define RRTHELPER_H

#include <openrave/openrave.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <time.h>
#include <stdlib.h>

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

typedef std::vector<float> configuration;
typedef std::vector<configuration> configSet;

class RRTNode
{
public:
    RRTNode();
    RRTNode(configSet configSetIn);
    ~RRTNode();

    bool SameNode(RRTNode* checkNode);
    void SetConfig(configSet config);
    configSet GetConfig();
    void AddStep(configSet s);

    void SetParent(RRTNode*);
    RRTNode* GetParent();
//  double disToNode(configuration config);

private:
    configSet configSet_;
    RRTNode* parent_;
};

typedef std::vector<RRTNode*> tree;

class NodeTree
{
public:
    NodeTree();
    NodeTree(RRTNode* initNode);
    ~NodeTree();

    RRTNode* GetRoot();

    bool InTree(RRTNode* testNode);
    bool Add(RRTNode* growNode);
    bool Remove(RRTNode *remNode);
    RRTNode* GetNode(int index);
    int TreeSize();
    int GetIndex(RRTNode* findNode);
    bool IsRoot(RRTNode* checkNode);

    std::vector<RRTNode *> GetPath(RRTNode *goal);

    // initialize geodesic_// NEW IN PROJECT
 //   void SetNumGrippers(int num);
 //   void InitGeodesic(configuration initCofig);

private:
    tree tree_;
    RRTNode* rootNode_;
//    int numGrippers_;

    // NEW IN PROJECT
    // the configuration of geodesic
    // geodesic of (i,j) stored in i*n+j
    // n is the number of gripper; sizeof geodesic_ = n^2
};

// NEW IN PROJECT
class ParameterSet
{
public:
    ParameterSet();
//    ~ParameterSet();

    // To simplify the problem, I assume all grippers having the same boundary
    std::vector<double> lowerBound;
    std::vector<double> upperBound;
    configuration weight;
    // NEW IN PROJECT
    int numGrippers;
    configSet geodesicConfig;
    std::vector<float> geodesic;

    // Geodesic of (i,j) gripper stored in i*n+j; n is num of grippers
    void InitGeodesic(configSet initConfig);
};

static ParameterSet plannerSetup;

///////////////////////////////////// Some Helper Function ///////////////
///
///

int RandNode(int size);

configuration ScaleConfig(configuration v, float scale);

void ConfigPrintHelp(configuration config);

configuration VecToConfig(std::vector<double> vec);

std::vector<double> ConfigToVec(configuration config);
// if configuration violate joint limits
bool OutBound(configuration a);
bool SameDirection(configuration a, configuration b);

//void InitialWeight(configuration& w);

configuration SumConfig(configuration A, configuration B);
// A-B
configuration SubtractConfig(configuration A, configuration B);

float Distance(configuration A, configuration B);
float WeightedDis(configuration A, configuration B);

// Nearest Node on the tree // REVISED: INPUT CHANGED TO CONFIGSET
// RRTNode* NearestNode(NodeTree* treeA, configuration config);

// Extend the tree from the nearest node, to as close to the configuration as possible
RRTNode* ConnectTree(OpenRAVE::EnvironmentBasePtr env, OpenRAVE::RobotBasePtr robot, NodeTree &treeA, RRTNode* nearestNode, configuration config);

// Sample with bias, bias is in #define
//configuration RandSample();

// return unitstep vector, the step size is in #define, could be adjust later
configuration UnitStep(configuration start, configuration goal, float stepSize);

/////////// New in this project
///
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

// Sample configuration
configuration RandSample();

// Sampe configSet
configSet SampleSet(int gripperSize);

configSet SampleSetWithBase(configuration baseSample, int index);

// return unitstep vector, the step size is in #define, could be adjust later
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

//////////////////////////////////// global variable ////////////////////////
/*
static std::vector<double> lowerBound;
static std::vector<double> upperBound;
static configuration weight;
// NEW IN PROJECT
static int numGrippers;
*/

//static ParameterSet plannerSetup;

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// CPP //////////////////////////////////////////
/// //////////////////////////////////////////////////////////////////////////
///


////////////////////// RRTNode Class ///////////////////////////
/// \brief RRTNode::RRTNode
/// \param config
///
RRTNode::RRTNode(){ parent_=NULL; }

RRTNode::RRTNode(configSet configSetIn):configSet_(configSetIn)
{
    parent_=NULL;
}

RRTNode::~RRTNode()
{
    configSet_.clear();
    parent_ = NULL;
}


void RRTNode::SetConfig(configSet config)
{
    configSet_ = config;
}


configSet RRTNode::GetConfig()
{
    return configSet_;
}

RRTNode* RRTNode::GetParent()
{
    return parent_;
}

void RRTNode::AddStep(configSet s)
{
    configSet_=SetSumConfig(configSet_,s);
}

void RRTNode::SetParent(RRTNode* parentNode)
{
    parent_ = parentNode;
//    parentNode->SetChild(this);
}

bool RRTNode::SameNode(RRTNode *checkNode)
{
    configSet config = checkNode->GetConfig();
    float error = 0;

    for(int i =0; i<config.size(); i++)
    {
        float ele_error = WeightedDis(config.at(i),configSet_.at(i)) ;
        error = error+ele_error;
    }

    if (error<ERRORTHRESHOLD){ return true; }
    return false;
}


///////////////////// NodeTree Class ///////////////////////////
/// \brief NodeTree::NodeTree
/// \param initNode
///
NodeTree::NodeTree(){}

NodeTree::NodeTree(RRTNode *initNode):rootNode_(initNode)
{
    tree_.push_back(rootNode_);    
}

NodeTree::~NodeTree()
{

    tree_.clear();
}

RRTNode* NodeTree::GetRoot()
{
    return rootNode_;
}

bool NodeTree::InTree(RRTNode* testNode)
{
    for (std::vector<RRTNode*>::iterator i=tree_.begin(); i!=tree_.end(); i++)
    {
        if((*i)->SameNode(testNode)){ return true; }
    }
    return false;
}

// double check this function later, for parent pointer issue
bool NodeTree::Add(RRTNode* growNode)
{
    if (!InTree(growNode))
    {
        tree_.push_back(growNode);
        return true;
    }
    return false;
}

bool NodeTree::Remove(RRTNode* remNode)
{
    if (tree_.size()<=1)
    {
        tree_.clear();
        return false;
    }
    for(tree::iterator i = tree_.begin()+1; i!=tree_.end(); i++)
    {
        if(((*i)->GetParent())->SameNode(remNode))
            (*i)->SetParent(remNode->GetParent());
        if(remNode->SameNode(*i))
            tree_.erase(i);
    }
    return true;
}

RRTNode* NodeTree::GetNode(int index)
{
    if(index<tree_.size())
        return tree_.at(index);
    return rootNode_;
}

int NodeTree::TreeSize()
{
    return tree_.size();
}

int NodeTree::GetIndex(RRTNode * findNode)
{
    for (int i = 0; i<tree_.size(); i++){
        if(findNode->SameNode(tree_.at(i)))
            return i;
    }
    return -1;
}

bool NodeTree::IsRoot(RRTNode *checkNode)
{
    if(checkNode->SameNode(rootNode_))
        return true;
    return false;
}

std::vector<RRTNode*> NodeTree::GetPath(RRTNode* goal)
 {
     std::vector<RRTNode*> path;
     RRTNode* nodePtr = goal;
     while(!IsRoot(nodePtr)){
         path.insert(path.begin(),nodePtr);
         nodePtr = nodePtr->GetParent();
     }
     path.insert(path.begin(),nodePtr);
     return path;
 }


/////////////////////// ParameterSet Class /////////////////////////////
// the config here is in object configuration space, with x,y,z
// geodesicConfig dimension: 3*n
ParameterSet::ParameterSet(){}

void ParameterSet::InitGeodesic(configSet initConfig)
{
    geodesicConfig=initConfig;

    if (numGrippers == 0) {numGrippers = initConfig.size();}

    for (int i=0; i<numGrippers; i++)
    {
        for (int j=0; j<numGrippers; j++)
        {
        // initConfig is in p (object space)
            float dis = Distance(initConfig.at(i),initConfig.at(j) ) ;
            geodesic.push_back(dis);
        }
    }

}


 ////////////////////////////////// Helper function //////////////////////////

 int RandNode(int size)
 {
     int index = rand()%size;
     return index;
 }

 configuration ScaleConfig(configuration v, float scale)
 {
     configuration a;
     for (int i =0; i<v.size(); i++)
     {
         a.push_back(v.at(i)/scale);
     }
     return a;
 }

 void ConfigPrintHelp(configuration config){
     cout << "config is " << endl;
     for (int i =0; i<config.size();i++)
     {
         cout << config.at(i) << endl;
     }
 }


 configuration VecToConfig(std::vector<double> vec)
 {
     configuration config;
     for (int i=0; i<vec.size(); i++)
     {
         config.push_back(vec.at(i));
     }
     return config;
 }

 std::vector<double> ConfigToVec(configuration config)
 {
     std::vector<double> vec;
     for (int i=0; i<config.size();i++)
         vec.push_back(config.at(i));
     return vec;
 }


 bool OutBound(configuration a)
 {
     for (int i=0; i<a.size(); i++)
     {
         if (a.at(i)<plannerSetup.lowerBound.at(i)||a.at(i)>plannerSetup.upperBound.at(i))
             return true;
     }
     return false;
 }

 bool SameDirection(configuration a, configuration b){
    double result = 0;
    for (int i =1; i<a.size(); i++)
    {
        result = result +a.at(i)+b.at(i);
    }
    if(result > 0)
        return true;
    return false;
}

 /*
 void InitialWeight(configuration& w, configuration wIn)
 {
     w.clear();
     for (int i=0; i<w.size(); i++)
     {
         w.push_back(wIn.at(i));
     }
 }
*/

 configuration SumConfig(configuration A, configuration B)
 {
     configuration sum;
     for (int i=0; i<A.size(); i++)
     {
         sum.push_back(A.at(i)+B.at(i));
     }
     return sum;
 }

 configuration SubtractConfig(configuration A, configuration B)
{
    configuration sum;
    for (int i=0; i<A.size(); i++)
    {
        sum.push_back(A.at(i)-B.at(i));
    }
    return sum;
}

// euclidian distance
 float Distance(configuration A, configuration B)
 {
     float distance = 0;
     for(int i =0; i<A.size(); i++)
     {
         float ele_error;
         ele_error = std::pow(A.at(i)-B.at(i),2);
         distance = distance+ele_error;
     }
     return std::sqrt(distance);
 }

 float WeightedDis(configuration A, configuration B)
 {
     float distance = 0;
     for(int i =0; i<A.size(); i++)
     {
         float ele_error;
         ele_error = plannerSetup.weight.at(i)*std::pow(A.at(i)-B.at(i),2);
         distance = distance+ele_error;
     }
     return std::sqrt(distance);

 }

// Old version of nearestNode
/*
 RRTNode* NearestNode(NodeTree* treeA, configSet config)
 {
     RRTNode* nodePtr=treeA->GetRoot();
     RRTNode* minNode = nodePtr;
     int index=0;
     float minDis=SetWeightedDis(nodePtr->GetConfig(),config);
     for(int i=0; i<treeA->TreeSize(); i++)
     {
         nodePtr = treeA->GetNode(i);
         float curDis = SetWeightedDis(nodePtr->GetConfig(),config);
         if(curDis<minDis)
         {
             index =i;
             minNode = nodePtr;
             minDis = curDis;
         }
     }
     return treeA->GetNode(index);
 }
*/


 RRTNode* NearestNode(NodeTree* treeA, configSet config)
 {
     RRTNode* nodePtr=treeA->GetRoot();
     RRTNode* minNode = nodePtr;
     int index=0;
     float minDis=SetWeightedDis(nodePtr->GetConfig(),config);
     for(int i=0; i<treeA->TreeSize(); i++)
     {
         nodePtr = treeA->GetNode(i);
         float curDis = SetWeightedDis(nodePtr->GetConfig(),config);
         if(curDis<minDis)
         {
             index =i;
             minNode = nodePtr;
             minDis = curDis;
         }
     }
     return treeA->GetNode(index);
 }

 configuration RandSample()
 {
     configuration randConfig;
     for (int i=0; i<plannerSetup.lowerBound.size(); i++)
     {
         float randNum = (float)rand();
         randNum =randNum/(float)RAND_MAX *(plannerSetup.upperBound.at(i)-plannerSetup.lowerBound.at(i))+plannerSetup.lowerBound.at(i);
         randConfig.push_back(randNum);
     }
     return randConfig;
 }

 configuration UnitStep(configuration start, configuration goal, float stepSize)
 {
     configuration step;
     float length = Distance(start, goal);
     for (int i=0; i<start.size(); i++)
     {
         step.push_back((goal.at(i)-start.at(i))/length*stepSize*std::sqrt(plannerSetup.weight.at(i)));
     }
     return step;
 }




//////////////////////////////////// New / Revised function from HW3 for Project
///
///
void SetConfigPrintHelp(configSet config)
{
    for (int i=0; i<config.size(); i++)
    {   ConfigPrintHelp(config.at(i));    }
}

bool SetOutBound(configSet a)
{
    for(int i=0; i<a.size(); i++)
    { if(OutBound(a.at(i))) {return true;} }
    return false;
}

configSet SetScaleConfig(configSet v, float scale)
{
    configSet config;
    for(int i=0; i<v.size(); i++)
    {
        config.push_back(ScaleConfig(v.at(i),scale));
    }
    return config;
}


configSet SetVecToConfig(std::vector<std::vector<double> > setVec)
{
    configSet setConfig;
    for (int i=0; i<setVec.size(); i++)
    {
        setConfig.push_back(VecToConfig(setVec.at(i)));
    }
    return setConfig;
}

std::vector<std::vector<double> > SetConfigToVec(configSet setConfig)
{
    std::vector<std::vector<double> > setVec;
    for (int i=0; i<setConfig.size(); i++)
    {
        setVec.push_back(ConfigToVec(setConfig.at(i)));
    }
    return setVec;
}

float SetWeightedDis(configSet A, configSet B)
{
    float dis = 0;
    for (int i = 0; i< A.size(); i++)
    {
        dis = dis + WeightedDis(A.at(i), B.at(i));
    }
    return dis;
}

configSet SetSumConfig(configSet A, configSet B)
{
    configSet setSum;
    for (int i =0; i< A.size(); i++)
    {
        setSum.push_back(SumConfig(A.at(i), B.at(i)));
    }
    return setSum;
}

// A-B
configSet SetSubtractConfig(configSet A, configSet B)
{
    configSet SetDiff;
    for (int i=0; i<A.size(); i++)
    {
        SetDiff.push_back(SubtractConfig(A.at(i), B.at(i)));
    }
    return SetDiff;
}

// Sampe configSet
configSet SampleSet(int gripperSize)
{
    configSet SetSample;
    for(int i =0; i<gripperSize; i++)
    {
        SetSample.push_back(RandSample());
    }
    return SetSample;
}

// No guarantee within violation still though
configSet SampleSetWithBase(configuration baseSample, int index)
{
    configSet setSample;
    for (int i=0; i<plannerSetup.numGrippers; i++)
    {
        if(i==index){setSample.push_back(baseSample);}
        else
        {
            // This sampling approach only works for the free gripper in space
            float radius = plannerSetup.geodesic.at(i*plannerSetup.numGrippers+index);
            configuration randConfig;
            for (int j = 0; j<baseSample.size(); j++)
            {
                float randNum = (float)rand();
                randNum =randNum/(float)RAND_MAX *2*radius+baseSample.at(j)-radius;
                randConfig.push_back(randNum);
            }
            setSample.push_back(randConfig);
        }
    }
    return setSample;
}

// return unitstep vector, the step size is in #define, could be adjust later
configSet SetUnitStep(configSet start, configSet goal, float stepSize)
{
    configSet setStep;
    for (int i=0; i<start.size(); i++)
    {
        setStep.push_back(UnitStep(start.at(i), goal.at(i), stepSize));
    }
    return setStep;
}

configuration RaveVecToConfig(OpenRAVE::Vector raveVec)
{
    configuration pos;
    double x=raveVec.x;
    pos.push_back(x);
    double y=raveVec.y;
    pos.push_back(y);
    double z=raveVec.z;
    pos.push_back(z);
    return pos;
}

// Only for config.size = 3, Vector = tran
OpenRAVE::Vector ConfigToRaveVec(configuration config)
{
    OpenRAVE::Vector raveVec;
    raveVec.x = config.at(0);
    raveVec.y = config.at(1);
    raveVec.z = config.at(2);
//    raveVec.w = 0.0;
    return raveVec;
}


OpenRAVE::Vector GetObjectPos(OpenRAVE::RobotBasePtr robotPtr)
{
    std::vector<KinBody::LinkPtr> iLPtr = robotPtr->GetLinks();
    Transform itrans = (iLPtr.at(6))->GetTransform();
    OpenRAVE::Vector ipos = itrans.trans;
    return ipos;
}

bool ConstraintViolation(configSet sampleConfig, vector<OpenRAVE::RobotBasePtr> robots, OpenRAVE::EnvironmentBasePtr env)
{
    for (int i=0; i<sampleConfig.size(); i++)
    {
        for (int j=i; j<sampleConfig.size(); j++)
        {

            float disMax = plannerSetup.geodesic.at(i*plannerSetup.numGrippers+j)+STRETCHING_THRESHOLD;
            // from sample config to object config
            robots.at(i)->SetActiveDOFValues(ConfigToVec(sampleConfig.at(i)));
            robots.at(j)->SetActiveDOFValues(ConfigToVec(sampleConfig.at(j)));

            OpenRAVE::Vector iPosRave = GetObjectPos(robots.at(i));
            OpenRAVE::Vector jPosRave = GetObjectPos(robots.at(j));

            configuration iPos = RaveVecToConfig(iPosRave);
            configuration jPos = RaveVecToConfig(jPosRave);

            if (Distance(iPos,jPos) > disMax)
            { return true; }

            // Collision checking
//            OpenRAVE::CollisionCheckerBasePtr collisionCheck;
//            env->SetCollisionChecker(collisionCheck);

            // ERROR HERE, RAY COLLISION CHECK SHOULD BE FIXED
//            OpenRAVE::RAY rayCheck(ConfigToRaveVec(iPos), ConfigToRaveVec(SubtractConfig(jPos,iPos)));
            OpenRAVE::RAY rayCheck;
            rayCheck.pos = GetObjectPos(robots.at(i))+0.2*(GetObjectPos(robots.at(j))-(GetObjectPos(robots.at(i))));
            rayCheck.dir= 0.5*(GetObjectPos(robots.at(j))-(GetObjectPos(robots.at(i))));
            vector<OpenRAVE::KinBodyPtr> kinbody;
            env->GetBodies(kinbody);

            for (int kin = 0; kin < kinbody.size(); kin++ )
            {
                if (env->CheckCollision(rayCheck,kinbody.at(kin)))
                {
                    double r = rayCheck.dir.lengthsqr2();
//                    cout<<"ray collision : " << r << endl;
                    return true;
                }
            }

//            if (collisionCheck->CheckCollision(rayCheck)) {return true;}
//            if (OpenRAVE::CollisionCheckerBase::CheckCollision(rayCheck)) {return true;}
        }
    }
    return false;
}


// Return true if collision happens
bool SetCollisionCheck(vector<RobotBasePtr> robots, EnvironmentBasePtr env)
{
    for (int i =0; i<robots.size(); i++)
    {
        if(env->CheckCollision(robots.at(i))||robots.at(i)->CheckSelfCollision())
        { return true; }
    }
    return false;
}



 ///////////////////////////////////// RRT Planning //////////////////////////
 /// \brief RRTPlanning
 /// \param env
 /// \param startConfig
 /// \param goalConfig
 /// \return
 ///

 std::vector<RRTNode*> RRTPlanning(OpenRAVE::EnvironmentBasePtr env,
     configSet goalConfig, float sampleBias, float stepSize)
 {
     vector<OpenRAVE::RobotBasePtr> robots;
     env->GetRobots(robots);

     int numGripper = robots.size();
     plannerSetup.numGrippers=numGripper;

//     OpenRAVE::RobotBasePtr robot = robots.at(0);
     std::vector<std::vector<double> > startSet;

     for (int i =0; i<numGripper; i++)
     {
         std::vector<double> startVec;
         robots.at(i)->GetActiveDOFValues(startVec);
         startSet.push_back(startVec);
     }

     RRTNode *startPtr;
     RRTNode startNode(SetVecToConfig(startSet));
     startPtr = new RRTNode(startNode);
     RRTNode* nearestNode;

     NodeTree *treePtr;
     treePtr = new NodeTree(startPtr);

     bool sampleGoal = false;

     RRTNode *tempNode;
     RRTNode* stopPtr;
     configSet stepConfig;
     float curStep=stepSize;

     bool findSol=false;
     int count = 0;

     // Randseed
     srand(time(NULL));

/*
     cout << "At the very beginning, Tree node (start) at:......!!!!!!!!" << endl;
     SetConfigPrintHelp(treePtr->GetRoot()->GetConfig());
     cout << "Start from robot:++++" << endl;
     SetConfigPrintHelp(SetVecToConfig(startSet));
*/
     while((!findSol)&&(count<1000000))
     {
//         if(count%100==1) {cout << "ith enter:------------------ "; cout << count << endl;}

         curStep = stepSize;
         float toss = (float)rand()/((float)RAND_MAX);
         configSet sampleConfig;
         configSet objectSampleConfig;

         if(toss<sampleBias)
         {
             sampleGoal = true;
             sampleConfig = goalConfig;
//             cout << "sample goal!!!!" << endl;
         }

         else
         {
//             sampleConfig = SampleSet(numGripper);
             sampleConfig = SampleSetWithBase(RandSample(),0);
             // Constraint violation & collision status for ray across end effectors
             while (ConstraintViolation(sampleConfig,robots, env))
             {
//                 cout << "!! ConstraintViolation !!" << endl;
//                 sampleConfig = SampleSet(numGripper);
                 sampleConfig = SampleSetWithBase(RandSample(),0);
             }

//             ConfigPrintHelp(sampleConfig.at(0));
             sampleGoal=false;
         }

         nearestNode = NearestNode(treePtr, sampleConfig);
/*
         cout << "Tree node (start) at:......" << endl;
         SetConfigPrintHelp(treePtr->GetRoot()->GetConfig());
         cout << "Nearest Node at:......" << endl;
         SetConfigPrintHelp(nearestNode->GetConfig());
         cout << "sampleConfig at:......" << endl;
         SetConfigPrintHelp(sampleConfig);
*/

         /// Not use connect, move it here

         stopPtr = nearestNode;
         // if the sampling is at nearest node, return nearest node.

         if (!(SetWeightedDis(nearestNode->GetConfig(), sampleConfig)<ERRORTHRESHOLD))
         {
//             cout << SetWeightedDis(nearestNode->GetConfig(), sampleConfig) << endl;

//             SetUnitStep(nearestNode->GetConfig(),sampleConfig, stepSize);
             stepConfig = SetUnitStep(nearestNode->GetConfig(),sampleConfig, stepSize);
             bool withinStep = false;

             if((SetWeightedDis(stopPtr->GetConfig(),sampleConfig))<STEPSIZE) {withinStep=true;}

             int inner_count =0;
             while(inner_count<30)
             {
                 inner_count++;
                 tempNode = new RRTNode(stopPtr->GetConfig());
                 tempNode->AddStep(stepConfig);
//                 cout << "***************** " << inner_count << " ************" << endl;
//                 ConfigPrintHelp(tempNode->GetConfig().at(0));
//                 ConfigPrintHelp(tempNode->GetConfig().at(1));

                 if (SetOutBound(tempNode->GetConfig()))
                 {                    
                     delete tempNode;
//                     if (sampleGoal) { cout << "out of bound!!" << endl;}
                     tempNode = stopPtr;
                     break;
                 }
                 if (ConstraintViolation(tempNode->GetConfig(),robots,env))
                 {
//                     cout << "!! ConstraintViolation !!" << endl;
                     delete tempNode;
//                     if (sampleGoal) { cout << "out of bound!!" << endl;}
                     tempNode = stopPtr;
                     break;
                 }

                 for (int irob=0; irob<robots.size(); irob++)
                 {
                     robots.at(irob)->SetActiveDOFValues(ConfigToVec((tempNode->GetConfig()).at(irob)));
                 }

                 if(SetCollisionCheck(robots, env))
                 {
                     if (curStep>0.1)
                     {
                         curStep=curStep/2.0;
                         // should be scaled
                         stepConfig = SetScaleConfig(stepConfig,2.0);
                         delete tempNode;
                         continue;
                     }
                     else
                     {
                        delete tempNode;
 //                       if (sampleGoal) { cout << "Collision!!" << endl;}
                        tempNode = stopPtr;
                        break;
                     }
                 }

                 // step; revise: no if before
                 if ((SetWeightedDis(stopPtr->GetConfig(),sampleConfig))>stepSize)
                 {
                     tempNode->SetParent(stopPtr);
                     stopPtr = tempNode;
                     treePtr->Add(tempNode);
//                     cout << "Stepping!!" << endl;
                 }

                 if ((SetWeightedDis(stopPtr->GetConfig(),sampleConfig))<stepSize)
                 {
//                     cout << "distance smaller than step size!!" << endl;
                     if (sampleGoal)
                     {
                         RRTNode* show = NearestNode(treePtr, goalConfig);

                         tempNode = new RRTNode (goalConfig);
                         tempNode->SetParent(stopPtr);
                         treePtr->Add(tempNode);
                         findSol = true;

                         show = NearestNode(treePtr, goalConfig);
                     }  // if sample is goal, connect directly
                     else
                     {
                         for (int irob=0; irob<robots.size(); irob++)
                         {
                             robots.at(irob)->SetActiveDOFValues(ConfigToVec((tempNode->GetConfig()).at(irob)));
                         }
                         if(!(SetCollisionCheck(robots, env)))
                         {
                             tempNode = new RRTNode (sampleConfig);
                             tempNode->SetParent(stopPtr);
                             treePtr->Add(tempNode);
//                             cout << "reach nearest point, successfully connect!" << endl;
                         }
                     } // if sample is not goal, check if not collision, connect
                     break;
                 }                 
             } // loop to step
         }

         /// ///// connect end /////////////
         count++;
     }// End of large loop
     cout << "RRT sampling size:" ;
     cout << treePtr->TreeSize() << endl;

     std::vector<RRTNode*> path;  // debug line
     if (treePtr->TreeSize()>1)
     { path = treePtr->GetPath(tempNode);}
//     else { std::vector<RRTNode*> path;}  // debug line

     float pathLength = 0;
     for (int i = 1; i<path.size(); i++)
     { pathLength = pathLength+SetWeightedDis((path.at(i-1))->GetConfig() ,(path.at(i))->GetConfig()); }
     cout << "RRT path size : ";
     cout << path.size() << endl;

     return path;

 }




std::vector<RRTNode*> SmoothPath(EnvironmentBasePtr env, std::vector<RRTNode *> &path,float stepSize, int iteration)
{
    int index1, index2; // checking node index, 2 is closer to goal
    RRTNode* node1;
    RRTNode* node2;
    RRTNode* tempNode;
    RRTNode* stopNode;
    bool insert = true;

    cout << "treeSize before smooth:" << endl;
    cout << path.size()<< endl;
    for (int it = 0; it<iteration; it++ )
    {
        vector<OpenRAVE::RobotBasePtr> robots;
        env->GetRobots(robots);
//        OpenRAVE::RobotBasePtr robot = robots.at(0);

        index1 = RandNode(path.size());
        index2 = RandNode(path.size());
        while(index1==index2) { index2 = RandNode(path.size()); }
        if(index1>index2) { int a = index1; index1 = index2; index2 = a; }

        node1=path.at(index1);
        node2=path.at(index2);
        configSet stepConfig = SetUnitStep(node1->GetConfig(),node2->GetConfig(),stepSize);

        std::vector<RRTNode*> tempPath;
        tempPath.clear();
        tempPath.push_back(node1);
        stopNode = node1;
        insert = true;

        // step from node 1 to node 2
        while (SetWeightedDis(stopNode->GetConfig(),node2->GetConfig())>stepSize)
        {
            tempNode = new RRTNode(SetSumConfig(stopNode->GetConfig(),stepConfig));
            if (SetOutBound(tempNode->GetConfig())){
                delete tempNode;
                tempNode = stopNode;
                insert = false;
                break;
            }
//            robot->SetActiveDOFValues(ConfigToVec(tempNode->GetConfig()));
            for (int irob=0; irob<robots.size(); irob++)
            { robots.at(irob)->SetActiveDOFValues(ConfigToVec((tempNode->GetConfig()).at(irob)));}

//            if(env->CheckCollision(robot)||robot->CheckSelfCollision())
            if(SetCollisionCheck(robots,env))
            {
                delete tempNode;
                tempNode=stopNode;
                insert = false;
                break;
            }

            tempPath.push_back(tempNode);
            stopNode = tempNode;
        }

    }
    cout << "tree size after smooth: +++++++++++++";
    cout << path.size() <<endl;
    float pathLength = 0;
    for (int i = 1; i<path.size(); i++)
    { pathLength = pathLength+SetWeightedDis((path.at(i-1))->GetConfig() ,(path.at(i))->GetConfig()); }
    cout << "Smooth path length : ";
    cout << pathLength << endl;

    return path;
}



std::vector<RRTNode*> BiRRTPlanning(OpenRAVE::EnvironmentBasePtr env,
    configSet goalConfig, float sampleBias, float stepSize)
{
    vector<OpenRAVE::RobotBasePtr> robots;
    env->GetRobots(robots);
\
    int numGripper = robots.size();
    plannerSetup.numGrippers=numGripper;

    std::vector<std::vector<double> > startSet;

    for (int i =0; i<numGripper; i++)
    {
        std::vector<double> startVec;
        robots.at(i)->GetActiveDOFValues(startVec);
        startSet.push_back(startVec);
    }

    RRTNode* startPtr;
    startPtr = new RRTNode(SetVecToConfig(startSet));
    NodeTree treeA(startPtr);
    RRTNode* goalPtr;
    goalPtr = new RRTNode(goalConfig);
    NodeTree treeB(goalPtr);
    NodeTree* treeStepPtr;  // for extend a step
    NodeTree* treeConPtr;   // for connect

    configSet stepConfig;
    configSet sampleConfig;
    bool connect=false;
    float curStep = stepSize;
    bool sample=true;
    bool AisCon = true;

    RRTNode* stepStop;
    RRTNode* conStop;
    RRTNode* nearestNode;
    RRTNode* tempPtr;

    // Initialize rand seed
    srand(time(NULL));

    while(!connect)
    {
        if (treeA.TreeSize()>treeB.TreeSize())
        { treeStepPtr = &treeA; treeConPtr = &treeB; AisCon=false; }
        else { treeStepPtr = &treeB; treeConPtr = &treeA; AisCon=true; }
        sample=true;

        // Extend the stepping node for one step
        while(sample)
        {
            sampleConfig = SampleSet(numGripper);
//            sampleConfig = SampleSetWithBase(RandSample(),0);
            // Constraint violation & collision status for ray across end effectors
            while (ConstraintViolation(sampleConfig,robots, env))
            {
                 sampleConfig = SampleSet(numGripper);
//                sampleConfig = SampleSetWithBase(RandSample(),0);
            }

            nearestNode = NearestNode(treeStepPtr, sampleConfig);
            stepConfig = SetUnitStep(nearestNode->GetConfig(),sampleConfig,stepSize);
            stepStop = new RRTNode(SetSumConfig(nearestNode->GetConfig(), stepConfig));

            if(SetOutBound(stepStop->GetConfig()))
            {
                delete stepStop;
                continue;
            }

            for (int irob=0; irob<robots.size(); irob++)
            {
                robots.at(irob)->SetActiveDOFValues(ConfigToVec((tempPtr->GetConfig()).at(irob)));
            }

            if(SetCollisionCheck(robots, env))
            {
                delete stepStop;
                continue;
            }
            stepStop->SetParent(nearestNode);
            treeStepPtr->Add(stepStop);
            sample=false;
            break;
        }

        nearestNode = NearestNode(treeConPtr, stepStop->GetConfig());
        conStop = nearestNode;

        if(SetWeightedDis(conStop->GetConfig(),stepStop->GetConfig())<ERRORTHRESHOLD)
        {connect =true; break;}

        // Connect the connecting tree to the stepping node
        stepConfig = SetUnitStep(conStop->GetConfig(),stepStop->GetConfig(),stepSize);
        while(SetWeightedDis(conStop->GetConfig(),stepStop->GetConfig())>stepSize)
        {
            tempPtr = new RRTNode(SetSumConfig(conStop->GetConfig(),stepConfig));

            for (int irob=0; irob<robots.size(); irob++)
            {
                robots.at(irob)->SetActiveDOFValues(ConfigToVec((tempPtr->GetConfig()).at(irob)));
            }

            if(SetCollisionCheck(robots, env))
            {
                if(curStep>0.2)
                {
                    curStep = curStep/2;
                    stepConfig = SetScaleConfig(stepConfig,2);
                    delete tempPtr;
                    break;
                }
                else
                {
                    delete tempPtr;
                    tempPtr = conStop;
                    break;
                }
            }
            tempPtr->SetParent(conStop);
            conStop=tempPtr;
            treeConPtr->Add(tempPtr);
        }
        if(SetWeightedDis(conStop->GetConfig(),stepStop->GetConfig())<=stepSize)
        {
            connect=true;
          //  cout << "constop, step stop are: "<< endl;
          //  ConfigPrintHelp(conStop->GetConfig());
          //  ConfigPrintHelp(stepStop->GetConfig());
        }
    }

    cout << "Bi-RRT sampling size:" ;
    cout << treeStepPtr->TreeSize()+treeConPtr->TreeSize() << endl;

    std::vector<RRTNode*> pathS;
    std::vector<RRTNode*> pathE;
    if (AisCon)
    {
        pathS = treeConPtr->GetPath(conStop);
        pathE = treeStepPtr->GetPath(stepStop);
    }
    else
    {
        pathS = treeStepPtr->GetPath(stepStop);
        pathE = treeConPtr->GetPath(conStop);
    }
    for (int i =pathE.size(); i>0; i--)
    { pathS.push_back(pathE.at(i-1));  }

    float pathLength = 0;
    for (int i = 1; i<pathS.size(); i++)
    { pathLength = pathLength+SetWeightedDis((pathS.at(i-1))->GetConfig() ,(pathS.at(i))->GetConfig()); }
    cout << "Bi-RRT path length : ";
    cout << pathLength << endl;
    cout << "path size : ";
    cout << pathS.size() <<endl;

    return pathS;

}






#endif
