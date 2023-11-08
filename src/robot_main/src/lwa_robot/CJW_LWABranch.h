#ifndef CJW_LWABRANCH
#define CJW_LWABRANCH

#include "CJW_BranchBased.h"
#include "CJW_BranchLegType.h"
#include "CJW_BranchWheelType.h"
#include "CJW_BranchArmType.h"

#define JOINT_ERROR_P_UP   1
#define JOINT_ERROR_P_DOWN 2
#define JOINT_ERROR_V_UP   3
#define JOINT_ERROR_V_DOWN 4
#define JOINT_ERROR_A_UP   5
#define JOINT_ERROR_A_DOWN 6
#define JOINT_ERROR_F_UP   7
#define JOINT_ERROR_F_DOWN 8

const string Joint_Error_Name[9]={"RIGHT",
"P_UP","P_UP",
"V_UP","V_LOW",
"A_UP","A_LOW",
"F_UP","F_LOW"};
/********************************************complex structure********************************/
/********************************************************
the robot branch is the comlex type structure
  ******************************************************/
class CJW_LWABranch
{
public:
    /*************Constructor****************************************************************/
    CJW_LWABranch(void):
        LegState(&(ComStruct.BModel[BRANCH_LEG_MODEL])),
        WheelState(&(ComStruct.BModel[BRANCH_WHEEL_MODEL])),
        ArmState(&(ComStruct.BModel[BRANCH_ARM_MODEL]))
    {
        BranchModel=&LegState;
    }
    ~CJW_LWABranch(void){}
    virtual void CJW_Init(void);
    /************based on function**********************************************************/
    virtual void SetStruct(struct CJW_BranchStructure* input);
    virtual void GetStruct(struct CJW_BranchStructure* output);
    virtual void GetStructOneJStruct(int jointi,struct CJW_JointStruct* out);
    virtual void ShowStruct(void);
    virtual void ShowBranch(void);
    virtual void SetRootParent(struct CJW_JointStruct* root);
    virtual int  SetModel(int state);
    virtual int  GetModel(void);
    virtual int  SetState(double state);
    virtual double  GetState(void);
    virtual int  SetBodyN(int n);
    virtual int  GetBodyN(void);
    /***********virtual the branch function *******************************************************/
    //get the one joint struct
    virtual void GetOneJStructScrew(int jointi,double screw[6]);
    //check the Joint limiation
    virtual int CheckJointLimit(int* error);
    //get the all angle state
    virtual int GetALLAngleState(struct CJW_BasedJoint *out);
    virtual int GetALLAngleP(double *out);
    virtual int GetALLAngleV(double *out);
    virtual int GetALLAngleA(double *out);
    virtual int GetALLAngleF(double *out);
    //get the one angle state
    virtual int GetOneAngleState(int jointi,struct CJW_BasedJoint *out);
    virtual int GetOneAngleP(int jointi,double *out);
    virtual int GetOneAngleV(int jointi,double *out);
    virtual int GetOneAngleA(int jointi,double *out);
    virtual int GetOneAngleF(int jointi,double *out);
    //set the all all state//can not change the limit
    virtual int SetALLAngleState(struct CJW_BasedJoint *out);
    virtual int SetALLAngleP(double *out);
    virtual int SetALLAngleV(double *out);
    virtual int SetALLAngleA(double *out);
    virtual int SetALLAngleF(double *out);
    //set the one angle state
    virtual int SetOneAngleState(int jointi,struct CJW_BasedJoint *out);
    virtual int SetOneAngleP(int jointi,double *out);
    virtual int SetOneAngleV(int jointi,double *out);
    virtual int SetOneAngleA(int jointi,double *out);
    virtual int SetOneAngleF(int jointi,double *out);
    /***********virtual the branch modoel function *************************************************/
    //get the angle state
    virtual int GetModelAngleState(struct CJW_BasedJoint *out);
    virtual int GetModelAngleP(double *out);
    virtual int GetModelAngleV(double *out);
    virtual int GetModelAngleA(double *out);
    virtual int GetModelAngleF(double *out);
    //set the banch state
    virtual int SetModelAngleState(struct CJW_BasedJoint *in);
    virtual int SetModelAngleP(double *in);
    virtual int SetModelAngleV(double *in);
    virtual int SetModelAngleA(double *in);
    virtual int SetModelAngleF(double *in);
    //get the tip state
    virtual int GetModelTip(struct CJW_TipStruct* Tip0);
    virtual int GetModelTipG(double TipG[4][4]);
    virtual int GetModelTipP(double TipP[3]);
    virtual int GetModelTipV(double TipV[6]);
    virtual int GetModelTipA(double TipA[6]);
    virtual int GetModelTipF(double TipF[6]);
    //set the tip state
    virtual int SetModelTip(struct CJW_TipStruct* Tip0);
    virtual int SetModelTipG(double TipG[4][4]);
    virtual int SetModelTipP(double TipP[3]);
    virtual int SetModelTipV(double TipV[6]);
    virtual int SetModelTipA(double TipA[6]);
    virtual int SetModelTipF(double TipF[6]);
    //trans between the angle and the tip///the neccesarty for the new type branch
    virtual int   ModelAngleToAllBodyStateRoot(void);
    virtual struct CJW_BranchDynamicState  ModelGetDynamicStateRoot(void);//based on AngleToAllBodyStateRoot();
    virtual int   ModelGetJacobianTip(double *outJ);//based on AngleToAllBodyStateRoot();
    virtual int   ModelGetGravityComForJoint(double* bodyR,double *JointF);//based on AngleToAllBodyStateRoot();
    virtual int   ModelAnglePTransToTipG(void);//based on AngleToAllBodyStateRoot();
    virtual int   ModelAngleVTransToTipV(void);//based on AngleToAllBodyStateRoot();
    virtual int   ModelAngleFTransToTipF(void);
    virtual int   ModelTipGTransToAngleP(double BodyG[4][4]);
    virtual int   ModelTipVTransToAngleV(void);
    virtual int   ModelTipFTransToAngleF(void);//based on AngleToAllBodyStateRoot();
    /******************************************************************************************/

protected:
    string BranchName;
    struct CJW_BranchStructure ComStruct;
    CJW_BranchBased     *BranchModel;
    CJW_BranchLegType   LegState;
    CJW_BranchWheelType WheelState;
    CJW_BranchArmType   ArmState;

private:

};



#endif
