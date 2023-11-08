#ifndef CJW_BRANCHBASED
#define CJW_BRANCHBASED

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>

#include "CJW_Math.h"
#include "CJW_RigidBody.h"
#include "Force_Quadprog.h"

using namespace std;

/********************************************Based branch Class*****************************/
/********************************************************
the robot branch structure in the root frame without structure
the branch is N rigid body with three motion description
  ******************************************************/
class CJW_BranchBased
{
public:
    /*************Constructor****************************************************************/
    CJW_BranchBased(struct CJW_BranchModel* Input){TheStructure=Input;}
    ~CJW_BranchBased(void){}
    virtual void CJW_Init(void);
    /************based on function**********************************************************/
    //show the branch struct
    virtual void ShowStruct(void);
    //get state of this branch model
    virtual int  GetModel(void);
    //virtual int  SetModel(int input);
    virtual double  GetState(void);
    virtual int  SetState(double input);
    /***********virtual the interface for the branch model*********************************************************************/
    virtual void ShowBranch(void);
    //get the angle state
    virtual int GetAngleState(struct CJW_BasedJoint *out);
    virtual int GetAngleP(double *out);
    virtual int GetAngleV(double *out);
    virtual int GetAngleA(double *out);
    virtual int GetAngleF(double *out);
    //set the banch state
    virtual int SetAngleState(struct CJW_BasedJoint *out);
    virtual int SetAngleP(double *out);
    virtual int SetAngleV(double *out);
    virtual int SetAngleA(double *out);
    virtual int SetAngleF(double *out);
    //get the tip state
    virtual int GetTip(struct CJW_TipStruct* Tip0);
    virtual int GetTipG(double TipG[4][4]);
    virtual int GetTipP(double TipP[3]);
    virtual int GetTipV(double TipV[6]);
    virtual int GetTipA(double TipA[6]);
    virtual int GetTipF(double TipF[6]);
    //set the tip state
    virtual int SetTip(struct CJW_TipStruct* Tip0);
    virtual int SetTipG(double TipG[4][4]);
    virtual int SetTipP(double TipP[3]);
    virtual int SetTipV(double TipV[6]);
    virtual int SetTipA(double TipA[6]);
    virtual int SetTipF(double TipF[6]);
    /***********compute the state of the model branch for the robot*********************************************************************/
    //motion and dynamic function to improve the speed for all forward compute function
    virtual int  AngleToAllBodyStateRoot(void);
    //express function after running the function AngleToAllBodyStateRoot();
    virtual struct CJW_BranchDynamicState  GetDynamicStateRoot(void);
    virtual int  GetJacobianTip(double *outJ);
    virtual int  GetGravityComForJoint(double* bodyR,double *JointF);
    virtual int  AnglePTransToTipG(void);//trans between the angle and the tip
    virtual int  AngleVTransToTipV(void);//the velocity of tip is expressed in the tip coordinate
    virtual int  TipFTransToAngleF(void);//based on AngleToAllBodyStateRoot();!!!!!

    /*the inverse compute should be design of the new type branch********************************************************/
    virtual int  AngleFTransToTipF(void)=0;
    virtual int  TipGTransToAngleP(double BodyG[4][4])=0;
    virtual int  TipVTransToAngleV(void)=0;



protected:
    //public for the Branch Struct
    struct CJW_BranchModel* TheStructure;
    struct CJW_TipStruct Tip;
    //bodyG is the body group and bodyV is the veloccity on the root coordinate//when the joint angle velocity is one
    struct CJW_TipStruct BodyStateRoot[BRANCH_BODY_MAX];
    double JocabinRoot[6*BRANCH_BODY_MAX];
    struct CJW_BranchDynamicState DynamicStateRoot;

    CJW_Math<double> MathFun;

private:

};


#endif
