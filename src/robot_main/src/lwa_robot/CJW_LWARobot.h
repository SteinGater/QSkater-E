#ifndef CJW_LWAROBOT
#define CJW_LWAROBOT

#include "CJW_LWABranch.h"
#include "Force_Quadprog.h"
#include "CJW_KalmanFilter.h"

/***************************定义常量************************/
//define the robot state
#define ROBOT_MODEL_MIX            -1
#define ROBOT_MODEL_LEG            0
#define ROBOT_MODEL_WHEEL          1
//define branch model
#define BRANCH_LEG_MODEL           0
#define BRANCH_WHEEL_MODEL         1
#define BRANCH_ARM_MODEL           2
//define branch state
#define BRANCH_SWI_STATE           0
#define BRANCH_SUP_STATE           1
#define BRANCH_BOUNDING_STATE      0.5
//define the foot force limitation
#define FOOT_FZ_MAX                 300
#define FOOT_FZ_MIN                 10
#define FOOT_FXYZ_U                 0.3
#define FOOT_FXY_MAX                90
#define FOOT_FXY_MIN                -90

/****************************define***************************/
const string Branch_Model_Name[BRANCH_MODEL_MAX]={"LEG_MODEL","WHEEL_MODEL","ARM_MODEL"};




/********************************legged robot structure****************8***********************************/
class CJW_LWARobot
{
public:
    /*************Constructor****************************************************************/
    CJW_LWARobot(void){}
    ~CJW_LWARobot(void){}
    virtual void CJW_Init(void);

    /************based on function**********************************************************/
    virtual int SetStruct(struct CJW_RobotStructure* input);
    virtual int GetStruct(struct CJW_RobotStructure* output);
    virtual void ShowStruct(void);
    virtual void ShowMainBody(void);
    virtual void ShowBranch(void);
    virtual void ShowALLModelAngle(void);
    virtual void ShowALLTipAbsolute(void);
    /***********branch normal function *************************************************/
    virtual int GetBranchN(void);
    virtual int SetRobotModel(int model);
    virtual int GetRobotModel(void);
    virtual int SetBranchiModel(int legi,int state);
    virtual int GetBranchiModel(int legi);
    virtual int SetBranchiState(int legi,double state);
    virtual double GetBranchiState(int legi);
    virtual int SetBranchModel(int* state);
    virtual int GetBranchModel(int* state);
    virtual int SetBranchState(double* state);
    virtual int GetBranchState(double* state);
    //get the angle state
    virtual int CheckBranchiLimit(int legi,int* error);
    virtual int GetBranchiAngleState(int legi,struct CJW_BasedJoint *out);
    virtual int GetBranchiAngleP(int legi,double *out);
    virtual int GetBranchiAngleV(int legi,double *out);
    virtual int GetBranchiAngleA(int legi,double *out);
    virtual int GetBranchiAngleF(int legi,double *out);
    virtual int GetBranchiModelAngleState(int legi,struct CJW_BasedJoint *out);
    virtual int GetBranchiModelAngleP(int legi,double *out);
    virtual int GetBranchiModelAngleV(int legi,double *out);
    virtual int GetBranchiModelAngleA(int legi,double *out);
    virtual int GetBranchiModelAngleF(int legi,double *out);
    //set the banch state
    virtual int SetBranchiAngleState(int legi,struct CJW_BasedJoint *in);
    virtual int SetBranchiAngleP(int legi,double *in);
    virtual int SetBranchiAngleV(int legi,double *in);
    virtual int SetBranchiAngleA(int legi,double *in);
    virtual int SetBranchiAngleF(int legi,double *in);
    virtual int SetBranchiModelAngleState(int legi,struct CJW_BasedJoint *in);
    virtual int SetBranchiModelAngleP(int legi,double *in);
    virtual int SetBranchiModelAngleV(int legi,double *in);
    virtual int SetBranchiModelAngleA(int legi,double *in);
    virtual int SetBranchiModelAngleF(int legi,double *in);
    //get and get the tip state
    virtual int GetBranchiModelTip(int legi,CJW_TipStruct* Tip0);
    virtual int GetBranchiModelTipG(int legi,double TipG[4][4]);
    virtual int GetBranchiModelTipV(int legi,double TipV[6]);
    virtual int GetBranchiModelTipA(int legi,double TipV[6]);
    virtual int GetBranchiModelTipF(int legi,double TipF[6]);
    virtual int SetBranchiModelTip(int legi,CJW_TipStruct* Tip0);
    virtual int SetBranchiModelTipG(int legi,double TipG[4][4]);
    virtual int SetBranchiModelTipV(int legi,double TipV[6]);
    virtual int SetBranchiModelTipA(int legi,double TipA[6]);
    virtual int SetBranchiModelTipF(int legi,double TipF[6]);
    //trans between the angle and the tip
    virtual int  BranchiModelAngleToAllBodyStateRoot(int legi);
    virtual struct CJW_BranchDynamicState  BranchiModelGetDynamicStateRoot(int legi);
    virtual int  BranchiModelGetJacobianTip(int legi,double *outJ);
    virtual int  BranchiModelGetGravityComForJoint(int legi,double *JointF);
    virtual int  BranchiModelAnglePTransToTipG(int legi);
    virtual int  BranchiModelAngleVTransToTipV(int legi);
    virtual int  BranchiModelAngleFTransToTipF(int legi);
    virtual int  BranchiModelTipGTransToAngleP(int legi);
    virtual int  BranchiModelTipVTransToAngleV(int legi);
    virtual int  BranchiModelTipFTransToAngleF(int legi);

    /***********mainbody normal function**********************************************************************************/
    virtual int GetMainBodyState(struct CJW_JointStruct *out);
    virtual int SetMainBodyState(struct CJW_JointStruct *in);
    virtual int GetMainBodyR(double BodyR[3][3]);
    virtual int GetMainBodyP(double BodyP[3]);
    virtual int GetMainBodyG(double BodyG[4][4]);
    virtual int GetMainBodyV(double BodyV[6]);
    virtual int GetMainBodyw(double Bodyw[3]);
    virtual int GetMainBodyv(double Bodyv[3]);
    virtual int GetMainBodyA(double BodyA[6]);
    virtual int GetMainBodyAw(double BodyAw[3]);
    virtual int GetMainBodyAv(double BodyAv[3]);
    virtual int GetMainBodyF(double BodyF[6]);
    virtual int GetMainBodyFf(double BodyFf[3]);
    virtual int GetMainBodyFt(double BodyFt[3]);
    virtual int SetMainBodyR(double BodyR[3][3]);
    virtual int SetMainBodyP(double BodyP[3]);
    virtual int SetMainBodyG(double BodyG[4][4]);
    virtual int SetMainBodyV(double BodyV[6]);
    virtual int SetMainBodyw(double Bodyw[3]);
    virtual int SetMainBodyv(double Bodyv[3]);
    virtual int SetMainBodyA(double BodyA[6]);
    virtual int SetMainBodyAw(double BodyAw[3]);
    virtual int SetMainBodyAv(double BodyAv[3]);
    virtual int SetMainBodyF(double BodyF[6]);
    virtual int SetMainBodyFf(double BodyFf[3]);
    virtual int SetMainBodyFt(double BodyFt[3]);

    /***********robot normal function**********************************************************************************/
    //get the all angle state
    virtual int CheckALLLimit(void);
    virtual int GetALLAngleState(struct CJW_BasedJoint *out);
    virtual int GetALLAngleP(double *out);
    virtual int GetALLAngleV(double *out);
    virtual int GetALLAngleA(double *out);
    virtual int GetALLAngleF(double *out);
    //set the all all state
    virtual int SetALLAngleState(struct CJW_BasedJoint *out);
    virtual int SetALLAngleP(double *out);
    virtual int SetALLAngleV(double *out);
    virtual int SetALLAngleA(double *out);
    virtual int SetALLAngleF(double *out);
    /**based robot refresh************************************************************************************/
    /**based robot refresh from angle to the G V and dynamic state*******************************************/
    virtual int RobotAngleToAllStateBody(void);
    /**robot dynamic controller based on COI***************************/
    virtual int GetRobotDynamicStateBody(double COIInertia[36],double COIdInertia[36],double COIvelocity[6]);
    /*get the body exp force based on accelation using the COI dynamic***************/
    virtual int GetRobotDynamicBodyForce(double AimA[6],double outF[6]);
    virtual int GetBranchiDynamicTipForce(int legi,double AimA[6],double TipF[6]);//based on RobotAngleToAllStateBody();!!!!!
    //set the trans between angle and tip*****************************************************************/
    //get the absolute tip state
    virtual int GetMainModelTip(CJW_TipStruct* input);
    virtual int GetMainModelTipOne(int legi,CJW_TipStruct* input);
    virtual int GetMainModelTipGOne(int legi,double input[4][4]);
    virtual int GetMainModelTipVOne(int legi,double input[6]);
    virtual int GetMainModelTipAOne(int legi,double input[6]);
    virtual int GetMainModelTipFOne(int legi,double input[6]);
    //set the absolute tip state
    virtual int SetMainModelTip(CJW_TipStruct* input);
    virtual int SetMainModelTipOne(int legi,CJW_TipStruct input);
    virtual int SetMainModelTipGOne(int legi,double input[4][4]);
    virtual int SetMainModelTipVOne(int legi,double input[6]);
    virtual int SetMainModelTipAOne(int legi,double input[6]);
    virtual int SetMainModelTipFOne(int legi,double input[6]);
    //function of the robot
    virtual int  MainAnglePTransToTipGOne(int legi);//based on RobotAngleToAllStateBody();!!!!!
    virtual int  MainAngleVTransToTipVOne(int legi);//based on RobotAngleToAllStateBody();!!!!!
    virtual int  MainAngleFTransToTipFOne(int legi);
    virtual int  MainTipGTransToAnglePOne(int legi);
    virtual int  MainTipVTransToAngleVOne(int legi);
    virtual int  MainTipFTransToAngleFOne(int legi);//based on RobotAngleToAllStateBody();!!!!!
    virtual int  MainAnglePTransToTipG(void);
    virtual int  MainAngleVTransToTipV(void);//express the tip coordinate
    virtual int  MainAngleFTransToTipF(void);
    virtual int  MainTipGTransToAngleP(void);
    virtual int  MainTipVTransToAngleV(void);
    virtual int  MainTipFTransToAngleF(void);//express the tip coordinate

    /*****dynamic self function //based on RobotAngleToAllStateBody();!!!!!****************/
    virtual void MainResetPVKef(double dT,double* para);
    virtual int  MainTipPVTransToBodyPV(struct CJW_JointStruct* Ref,double dT);//estimation position of the body using ref data and supporting legs
    virtual int  GetExTipForceonQua(double ExpBodyF[6],double Qua_S[6],double Qua_W[3],double bodyF[6],double *outF);//based on RobotAngleToAllStateBody();!!!!!


    /**************wheel joint design and control*********************************************************************/
    virtual int  SetBranchWheelDirection(int branchi,double dir);
    virtual int  SetAllWheelDirection(double *dir);
    virtual double  SetBranchWheelDirAuto(int branchi,double exp_v[6]);
    virtual int  SetAllWheelDirAuto(double exp_v[6]);
    virtual int  SetBranchWheelSpeed(int branchi,double speed);
    virtual int  SetAllWheelSpeed(double *Speed);
    virtual int  GetBranchWheelDirection(int branchi,double* dir);
    virtual int  GetAllWheelDirection(double *dir);
    virtual int  GetBranchWheelSpeed(int branchi,double* speed);
    virtual int  GetAllWheelSpeed(double *Speed);

protected:
    string robotname;
    int BranchN;
    struct CJW_JointStruct MainBody;
    struct CJW_JointStruct BranchP[BRANCH_N_MAX];
    struct CJW_TipStruct   TipAbsolute[BRANCH_N_MAX];
    struct CJW_TipStruct   TipRoot[BRANCH_N_MAX];
    struct CJW_TipStruct   TipBody[BRANCH_N_MAX];
    struct CJW_BranchDynamicState DynamicStateBody;
    struct CJW_BranchDynamicState DynamicStateBranch[BRANCH_N_MAX];
    CJW_LWABranch BranchS[BRANCH_N_MAX];
    int RobotModel;
    /***************EKF of the robot body************/
    CJW_Leg_Body_KF<double,4> PV_KF_Estimator;

    CJW_Math<double> MathFun;

private:
    

};


#endif
