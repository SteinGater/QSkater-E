#ifndef CJW_RIGIDBODY
#define CJW_RIGIDBODY

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>
#include "ros/ros.h"
#include "CJW_RigidBody.h"

using namespace std;
/***************************定义常量************************/
#define RIGHT               0
#define ERROR               1

#define BRANCH_BODY_MAX     6
#define BRANCH_N_MAX        8
#define BRANCH_N_MAX3       24
#define BRANCH_MODEL_MAX    3


/********************************************************
one rigid robot description in its self frame
Inertial tensor:M
position and orietation of the Inertial ternsor:MG
body velocity screw: BodyV angular+linear
body accelation screw: BodyA angular+linear
body force    srcew: BodyF torque+linear force
  ******************************************************/
struct CJW_RigidBody
{
    //string name;
    double BodyV[6];
    double BodyA[6];
    double BodyF[6];
    double MG[4][4];
    double M[6][6];
};
struct CJW_TipStruct
{
    double G[4][4];
    double V[6];
    double A[6];
    double F[6];
};
/********************************************************
one norm joint description in parent frame
init position and orientation: G0
init screw in the parent frame: Screw0
parent body: *parent
child body: *child
  ******************************************************/
struct CJW_BasedJoint
{
    string name;
    double Angle;
    double Velocity;
    double Accelation;
    double Force;
    double AngleLimit[2];
    double VelocityLimit[2];
    double AccelationLimit[2];
    double ForceLimit[2];
};
struct CJW_JointOffset
{
    double direction;
    double AngleInit;
};

struct CJW_JointStruct
{
    string name;
    double G0[4][4];
    double Screw0[6];
    struct CJW_JointStruct *parent;
    struct CJW_RigidBody Body;
};

/***the robot branch structure*************************
 *the branch has BodyN body and joint
*********************************************************/
struct CJW_BranchModel
{
    int modelstate;
    int JointN;
    double State;
    struct CJW_BasedJoint* Joint[BRANCH_BODY_MAX];
    struct CJW_JointStruct JStruct[BRANCH_BODY_MAX+1];
};
struct CJW_BranchStructure
{
    string name;
    int BodyN;
    struct CJW_BasedJoint Joint[BRANCH_BODY_MAX];
    struct CJW_JointStruct JStruct[BRANCH_BODY_MAX];
    int Model_EN[BRANCH_MODEL_MAX];
    struct CJW_BranchModel BModel[BRANCH_MODEL_MAX];
};
struct CJW_BranchDynamicState
{
    double Inertia[36];
    double dInertia[36];
    double Momentum[6];
};

/********************************************************
the legged robot structure has
one MainBody
BranchN branch of the robot and its joint position
  ******************************************************/
struct CJW_RobotStructure
{
    string name;
    int BranchN;
    struct CJW_JointStruct MainBody;
    struct CJW_JointStruct BranchP[BRANCH_N_MAX];
    struct CJW_BranchStructure BranchBody[BRANCH_N_MAX];
};
/********************************************************/
extern struct CJW_RigidBody Virtual_Body;

/********************************************************
the gait struct for order move
  ******************************************************/
// struct CJW_GaitStructure
// {
//     struct CJW_JointStruct  Body;
//     struct CJW_TipStruct    Tip[BRANCH_N_MAX];
//     struct CJW_BasedJoint   Joint[BRANCH_N_MAX*BRANCH_BODY_MAX];
// };






/****************show the structure*********************************************************************/
/****************show the structure*********************************************************************/
/****************show the structure*********************************************************************/
void Print_CJW_RigidBody(struct CJW_RigidBody* input);
/********************************************************/
void Print_CJW_TipStruct(struct CJW_TipStruct* input);
/********************************************************/
void Print_CJW_BasedJoint(struct CJW_BasedJoint* input);
/********************************************************/
void Print_CJW_JointOffset(struct CJW_JointOffset* input);
/********************************************************/
void Print_CJW_JointStruct(struct CJW_JointStruct* input);
/********************************************************/
void Print_CJW_BranchModel(struct CJW_BranchModel* input);
/********************************************************/
void Print_CJW_BranchStructure(struct CJW_BranchStructure* input);
/********************************************************/
void Print_CJW_BranchDynamicState(struct CJW_BranchDynamicState* input);
/********************************************************/
void Print_CJW_RobotStructure(struct CJW_RobotStructure* input);



#endif
