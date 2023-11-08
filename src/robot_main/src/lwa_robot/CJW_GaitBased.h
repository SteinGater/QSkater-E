#ifndef CJW_GAITBASED
#define CJW_GAITBASED

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>

#include "CJW_Math.h"
#include "CJW_RigidBody.h"
#include "CJW_LWARobot.h"

/**************************************define the constant***********************************/
/*************************************************/
//define the type of one DOF design
#define DESIGN_ZERO             0
#define DESIGN_LINE             1
#define DESIGN_QUADRIC          2
#define DESIGN_CUBIC            3
#define DESIGN_QUARTIC          4
#define DESIGN_QUINTIC          5

#define DESIGN_SIN              100
#define DESIGN_TRAPEZIUM        101
#define DESIGN_CYCLOID          102


//define the position and orientation method{include SE(2) and SE(3)}
#define DESIGN_MOVE_COUPE       0
#define DESIGN_MOVE_DECOUPE     1

//define the Max velocity and accelation
#define DESIGN_MAX_V0          20
#define DESIGN_MAX_A0          20

//define the swing and supporting
#define DESIGN_SWI_STATE        0
#define DESIGN_SUP_STATE        1

/***********************************************the trajectory with control**********************************************/
//1.design the basis of the trajectory
//2.design the basis of the legged gait
class CJW_GaitBased
{
public:
    /*************Constructor****************************************************************/
    CJW_GaitBased(void){}
    ~CJW_GaitBased(void){}
    /************************************************************************************/
    virtual int  GetGaitN(void);
    virtual int  GetBranchiModel(int legi);
    virtual void SetBranchiModel(int legi,int model);
    virtual void GetBranchModel(int *out);
    virtual void SetBranchModel(int *in);
    virtual int  GetSwingOrder(int gaittype,int num,int* out);
    virtual int  GetWheelDirection(int gaittype,double* dir);
    /************************************************************************************/
    //design one DoF trajectory,the initial/end position,velocity,accelation is {start[3]/end[3]}
    //{T} is the total time, the t is real time,{Max[2]} is the maxmium velocity and accelation
    //{out[3]} is design position,velocity and accelation
    int Design_1DoF(int type,double start[3],double end[3],double T,double t,double out[3]);
    int Design_1DoF(int type,double start[3],double end[3],double T,double t,double out[3],double MaxV,double MaxA);
    int Design_1DoF_line(double start[3],double end[3],double T,double t,double out[3]);
    int Design_1DoF_cubic(double start[3],double end[3],double T,double t,double out[3]);
    int Design_1DoF_quintic(double start[3],double end[3],double T,double t,double out[3]);
    int Design_1DoF_sin(double start[3],double end[3],double T,double t,double out[3]);
    int Design_1DoF_trapezium(double start[3],double end[3],double T,double t,double out[3],double MaxV,double MaxA);
    int Design_1DoF_cycloid(double start[3],double end[3],double T,double t,double out[3]);
    /************************************************************************************/
    //define the type of SE3 design on screw method
    int Design_SE3(int mode,double type,double G1[4][4],double G2[4][4],double V1[6],double V2[6],double ACC1[6],double ACC2[6],
                       double MaxV[6],double MaxA[6],double T,double t,double Gout[4][4],double Vout[6],double Aout[6]);
    int Design_SO3(double type,double R1[3][3],double R2[3][3],double V1[3],double V2[3],double ACC1[3],double ACC2[3],
                       double MaxV[3],double MaxA[3],double T,double t,double Rout[3][3],double Vout[3],double Aout[3]);
    int Design_R3(double type,double P1[3],double P2[3],double V1[3],double V2[3],double ACC1[3],double ACC2[3],
                       double MaxV[3],double MaxA[3],double T,double t,double Pout[3],double Vout[3],double Aout[3]);
    int Design_SE3_ZERO(int mode,double type,double G1[4][4],double G2[4][4],double T,double t,double Gout[4][4],double Vout[6],double Aout[6]);
    int Design_SO3_ZERO(double type,double R1[3][3],double R2[3][3],double T,double t,double Rout[3][3],double Vout[3],double Aout[3]);
    int Design_R3_ZERO(double type,double P1[3],double P2[3],double T,double t,double Pout[3],double Vout[3],double Aout[3]);
    /************************************************************************************/
    //dir:运动方向和X正方向夹角（弧度制），每个周期线步长，每个周期角度步长
    int Design_SE2(int mode,double dir[3],double time,double outG[4][4]);
    /************************************************************************************/
    //define the type of one swing foot design;
    int  Design_swingfoot(int type,double start[3], double end[3], double T, double H, double time, double outP[3],double outV[3],double outA[3]);
    void Design_swingfoot_sin(double dL[2], double T, double H, double time, double outP[2],double outV[2],double outA[2]);
    void Design_swingfoot_trapezium(double dL[2], double T, double H, double time, double outP[2],double outV[2],double outA[2]);
    void Design_swingfoot_cycloid(double dL[2], double T, double H, double time, double outP[2],double outV[2],double outA[2]);
    /*************************************************************************************************************/
    /*************************************************************************************************************/
    /****************************the body robot gait desing for the cycle gait without control******************************/
    //平面路径规划dir_mode参数如下：运动方向和X正方向夹角（弧度制），每个周期线步长，每个周期角度步长
    //the OutTip V is not the TipV !!!!
    int Design_Gait_Cycle_Based(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0, double dir[3], double StepH,double T0,double time,
                                 int Movemode,int Bodytype,int Swingtype, double* SwingStart,double* SwingDuty,
                                 struct CJW_JointStruct* OutBody,struct CJW_TipStruct* OutTip,double* outstate);
    //the based coordinate is the floating coordinate for roller-skating gait
    //Step[0] is the Step Height ; Step[1] is the Step Length;Margins[0] is the body move length ; Margins[1] is the body orientation along X axis;
    //the OutTip V is not the TipV !!!!
    int Design_Skating_Cycle_Based(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0,double Margins[2],double Step[2],double T0,double time,
                                    int Movemode,int Bodytype,int Swingtype, double SwingStart,int* Swingflag,
                                    struct CJW_JointStruct* OutBody,struct CJW_TipStruct* OutTip,double* outstate);
    /****************************virtual gait for typical position legged gait***********************************************/
    virtual int Design_Leg_Position0(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0, double dir[3], double StepH,double T0,double time,
                                      int Movemode,int Bodytype,int GaitType,int Swingtype,
                                      struct CJW_JointStruct* OutBody,struct CJW_TipStruct* OutTip,double* outstate)=0;
    /****************************virtual gait for typical position arm design***********************************************/
    virtual int Design_Arm_Position0(int* arm_flag,struct CJW_TipStruct* Start, struct CJW_TipStruct* End,double T0,double time,int Movemode,
                                        struct CJW_TipStruct* OutTip,double* outstate)=0;
    /****************************virtual gait for typical position wheel design***********************************************/
    virtual int Design_Wheel_Position0(struct CJW_JointStruct* BodyStart, struct CJW_JointStruct* BodyEnd,double T0,double time,int Movemode,
                                        struct CJW_JointStruct* OutTip,double* outstate)=0;
    virtual int Design_Skating_Position0(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0,double Margins[2],double Step[2],double T0,double duty,double time,
                                    int Movemode,int Bodytype,int GaitType,int Swingtype,
                                    struct CJW_JointStruct* OutBody,struct CJW_TipStruct* OutTip,double* outstate)=0;

    /*************************************************************************************************************/
    /*************************************************************************************************************/
    /***************************adaptive gait for robot with the control*********************************************************/








    /*************************************************************************************************************/
    /*************************************************************************************************************/
    /**************************************transform between different robot model*************************************************/



protected:
    int GaitN;
    int GaitBranchModel[BRANCH_N_MAX];
    CJW_Math<double> MathFun;

private:
    
};



#endif
