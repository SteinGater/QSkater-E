#ifndef CJW_GAITTOROBOTBASED
#define CJW_GAITTOROBOTBASED

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>
#include "CJW_LWARobot.h"
#include "CJW_GaitBased.h"
#include "CJW_Gait_Bip.h"
#include "CJW_Gait_Qua.h"
#include "CJW_Gait_Hex.h"

/**************************************define the constant***********************************/
//#define CONTROL_VELOCITY_PRE        0 //define the supporting velocity pre control in the force control model

//define the tip data in the self coordinate and special coordinate
#define TIP_DATA_SELF           0
#define TIP_DATA_SPE            1

/*******define the control mode***************/
#define CONTROL_MODE_POSITION       0
#define CONTROL_MODE_VELOCITY       1
#define CONTROL_MODE_FORCE          2


/*******define the adaptive walk control parameters***************/
//#define BODY_CONTROL_MAX_ACC        10
#define WALK_ENTER_SUP_TIME         0.05
#define WALK_OUT_SUP_TIME           0.05
#define WALK_ENTER_SWI_TIME         0.05
#define WALK_OUT_SWI_TIME           0.05
#define WALK_ENTER_SUP_H0           0.005

/*******define the adaptive skating control parameters***************/
#define SKATING_OUT_SWING_TIME      0.05
#define SKATING_ENTER_SUP_TIME      0.05
#define SKATING_PUSH_DEEP           0.00

/*******define the bounding parameters***************/
#define BOUNDING_Z_MIN              0.2
#define BOUNDING_Z_MAX              0.3
#define BOUNDING_SWING_T0           0.1

/*******error code of robot***************/
#define GAIT_ERROR_ROBOT_STATE      1
#define GAIT_ERROR_LEG_STATE        2
#define GAIT_ERROR_INV_POSITION     3
#define GAIT_ERROR_INV_VELOCITY     4
#define GAIT_ERROR_INV_FORCE        5

const string Gait_Error_Name[6]={"Right","robot state","leg state","inv position","inv velocity","inv force"};

/******************special design*****************************/
#define SKATING_TURNING_FLAG        1
/***********************************************define the gait for the robot**********************************************/
//1.design the controller
//2.contact the gait and robot
//3.design the special gait
class CJW_GaitToRobotBased
{
public:
    /*************Constructor****************************************************************/
    CJW_GaitToRobotBased(void){}
    ~CJW_GaitToRobotBased(void){}
    /*******************************contact the robot and gait***********************************/
    virtual int Init(CJW_LWARobot* exprobot,CJW_LWARobot* realrobot,CJW_GaitBased* gait);
    virtual void SetExpRobot(CJW_LWARobot* robot);
    virtual void SetRealRobot(CJW_LWARobot* robot);
    virtual void SetGait(CJW_GaitBased* gait);
    /*************************************************************************************************************/
    /*************************************************************************************************************/
    /*******************************controller of the robot**********************************************************************/
    virtual int SetControlParameter(double BodyFKp[6],double BodyFKd[6],double BodyVKd[6],double Qua_S[6],double Qua_W[3],double BodyAccMax[6],double BodyAccMin[6],
                                      double InsideKp[3],double InsideKd[3],double SwingKp[3],double SwingKd[3]);
    virtual int SetControlBodyFParameter(double BodyFKp[6],double BodyFKd[6]);
    virtual int PDController_F_BasedCOI(void);
    virtual int PController_V_BasedTip(void);
    virtual int Control_Robot_Based(int ControlStype);
    /*************************************************************************************************************/
    /*************************************************************************************************************/
    /*******************************gait of the robot************************************************************************/
    virtual int SetInitState(void);
    virtual int SetInitState(struct CJW_LWARobot* ThisRobot);
    virtual int SetInitState(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0);
    virtual int SetGaitMotionType(int gaittype,int movemode,int bodytype,int swingtype);
    virtual int SetGaitParameter(struct CJW_JointStruct *Body0,struct CJW_TipStruct* Tip0, double T0,double stepH,double* dir,double* margin);
    virtual int GetContactFlag(void);
    /******************gait for based on user************************************************/
    virtual int Design_Robot_Emergency(void);
    /******************stop move for LAW_robot*****************************************************/
    virtual int Design_Robot_Stop(void);
    virtual int Design_Robot_MapToInital(struct CJW_JointStruct *Body0,struct CJW_TipStruct* Tip0,
                                                    struct CJW_JointStruct *Bodyout,struct CJW_TipStruct* Tipout);
    //set the exp robot body GVA in body coordinate and the Tip G V A to the joint position and velocity
    //sepcial in the LEG and WHEEL model ,the Tip velcoity and accelation is the spcecial coordiante
    virtual int Design_BodyTipToJoint(struct CJW_JointStruct *Body,struct CJW_TipStruct* Tip,double* BranchiState,int* BranchiType);
    /******************tip move for LAW_robot*****************************************************/
    virtual int Design_MainTip_PPmode(double* State,double time);
    virtual int Design_MainTip_P0Test(double* State,double Para[6],double time);


    /*********************Auto choose the gait****************************************/
    int Design_Move_Refresh(double MAX_A_SE2[6]);
    int Design_Move_Run(int StartFlag,double time);


    /******************open gait *****************************************************************************/
    /******************legged gait for LAW_robot*****************************************************/
    virtual int Design_LegMove_Kinetic(double time);
    /******************trans bewteen legged and wheel for LAW_robot*****************************************************/
    virtual int Design_BetweenLegAndWheel_Kinetic(struct CJW_JointStruct *BodyMid,struct CJW_TipStruct* TipMid, int* leg_model,double time);
    /******************skating gait for LAW_robot*****************************************************/
    virtual int Design_Skating_Kinetic(double time); 
    /*********************bounding gait**************************************************/
    int Design_DoubleSLIP_Kinetic(double Zmax, double TJ, double ZJ, double thmax, double LB,
                              double time0,double* outT,double* outBody);
    int Design_Bounding_Kinetic(double time,int *flag);

    /******************adaptive gait *****************************************************************************/
    /******************adaptive walking gait *****************************************************************************/
    int Design_Walk_Adapt_Fore(struct CJW_JointStruct *BodyZ,struct CJW_JointStruct *BodyNow,double MAX_A_SE2[6],double TT,struct CJW_JointStruct *BodyNext);
    int Design_Walk_Adapt_Refresh(double MAX_A_SE2[6]);
    int Design_Walk_Adapt_Based(double time0);
    /******************dynamic adaptive skating gait(pace and trot)*****************************************************************************/
    int Design_Skate_Adapt_Refresh(void);
    int Design_Skate_Adapt_Based(double time);
    /********************static skating gait(swizzling and 3+1)**************************************************/
    int Design_Skate_Static_Refresh(void);
    int Design_Skate_Static_Based(double time);
    int Design_Skate_Swizzling_Refresh(void);
    int Design_Skate_Swizzling_Based(int Startflag,double time);


protected:
    /********based struct between the robot and gait*************/
    CJW_LWARobot*         ExpRobot;
    CJW_LWARobot*         RealRobot;
    CJW_GaitBased*          ThisGait;
    /********parameters of the gait command***********/
    struct CJW_JointStruct  ComBody;
    struct CJW_TipStruct    ComTip[BRANCH_N_MAX];
    int                     ComBraType[BRANCH_N_MAX];
    double                  ComBraState[BRANCH_N_MAX];
    struct CJW_JointStruct  Gait_Body;
    struct CJW_TipStruct    Gait_Tip[BRANCH_N_MAX];
    struct CJW_JointStruct  Gait_BodySE2;
    struct CJW_TipStruct    Gait_TipSE2[BRANCH_N_MAX];
    double                  Gait_T0;
    double                  Gait_StepH;
    //运动方向（与X方向夹角）+步长+步角
    double                  Gait_Dir[3];
    //轮滑侧向摆动距离+翻滚角度+推地占空比/轮滑相位+轮滑推地方向+轮滑被动轮偏航角
    double                  Gait_Margin[6];
    /********parameters of the robot gait*************/
    struct CJW_JointStruct  GaitInitBody0;
    struct CJW_TipStruct    GaitInitTip0[BRANCH_N_MAX];
    struct CJW_BasedJoint   GaitInitJoint0[BRANCH_N_MAX*BRANCH_BODY_MAX];
    struct CJW_JointStruct  GaitInitBody1;
    struct CJW_TipStruct    GaitInitTip1[BRANCH_N_MAX];
    struct CJW_BasedJoint   GaitInitJoint1[BRANCH_N_MAX*BRANCH_BODY_MAX];
    struct CJW_JointStruct  GaitInitBody2;
    struct CJW_TipStruct    GaitInitTip2[BRANCH_N_MAX];
    struct CJW_BasedJoint   GaitInitJoint2[BRANCH_N_MAX*BRANCH_BODY_MAX];
    int                     GaitNum;
    int                     GaitSwingflag[BRANCH_N_MAX];
    int                     GaitType;
    int                     MoveMode;
    int                     BodyType;
    int                     SwingType;
    /********parameters of the control*************/
    double                  ControlBodyFKp[6];
    double                  ControlBodyFKd[6];
    double                  ControlBodyAccMax[6];
    double                  ControlBodyAccMin[6];
    double                  ControlQua_S[6];
    double                  ControlQua_W[3];
    double                  ControlInsideKp[3];
    double                  ControlInsideKd[3];
    double                  ControlSwingKp[3];
    double                  ControlSwingKd[3];
    double                  ControlBodyVKp[6];
    /***************wheel design********************/
    double                  ExpWheelStructDir;
    double                  ExpWheelDir[BRANCH_N_MAX];
    double                  ExpWheelSpeed[BRANCH_N_MAX];

    CJW_Math<double> MathFun;
private:
    


};

#endif
