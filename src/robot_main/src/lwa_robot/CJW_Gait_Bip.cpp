#include "CJW_Gait_Bip.h"



/*******************************************************************************/
int  CJW_Gait_Bip::GetSwingOrder(int gaittype,int num,int* out)
{
    switch(num%BIPED_NUM)
    {
        case 0:out[0]=DESIGN_SUP_STATE;out[1]=DESIGN_SWI_STATE;break;
        case 1:out[0]=DESIGN_SWI_STATE;out[1]=DESIGN_SUP_STATE;break;
        default: return ERROR;
    }
    return RIGHT;
}
int  CJW_Gait_Bip::GetWheelDirection(int gaittype,double* dir)
{
    dir[0]=-1;
    dir[0]=1;
    return RIGHT;
}
/*****************************************type gait**************************************/
int CJW_Gait_Bip::Design_Leg_Position0(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0, double dir[3], double StepH,double T0,double time,
int Movemode,int Bodytype,int GaitType,int Swingtype,
struct CJW_JointStruct* OutBody,struct CJW_TipStruct* OutTip,double* outstate)
{
    double SwingStart0[BIPED_NUM]={0,0.5};
    double SwingDuty0[BIPED_NUM]={0.5,0.5};
    return Design_Gait_Cycle_Based(Body0,Tip0,dir,StepH,T0,time,Movemode,Bodytype,Swingtype,SwingStart0,SwingDuty0,OutBody,OutTip,outstate);
}
/****************************virtual gait for typical position arm design***********************************************/
int CJW_Gait_Bip::Design_Arm_Position0(int* arm_flag,struct CJW_TipStruct* Start, struct CJW_TipStruct* End,double T0,double time,int Movemode,
                                    struct CJW_TipStruct* OutTip,double* outstate)
{
    return ERROR;
}

/****************************virtual gait for typical position wheel design***********************************************/
int CJW_Gait_Bip::Design_Wheel_Position0(struct CJW_JointStruct* BodyStart, struct CJW_JointStruct* BodyEnd,double T0,double time,int Movemode,
                                    struct CJW_JointStruct* OutTip,double* outstate)
{
    return ERROR;
}

int CJW_Gait_Bip::Design_Skating_Position0(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0,double Margins[2],double Step[2],double T0,double duty,double time,
                                int Movemode,int Bodytype,int GaitType,int Swingtype,
                                struct CJW_JointStruct* OutBody,struct CJW_TipStruct* OutTip,double* outstate)
{
    //double SwingT0=0.5;
    int Swingf[BIPED_NUM]={0,1};
    double thetime=time;double theT0=T0/2;
    double theMargins[2]={-fabs(Margins[0]),fabs(Margins[1])};
    double theStep[2]={fabs(Step[0]),fabs(Step[1])};
    if(time<=0)
    {
        thetime=0;
    }
    else if(time<=theT0)
    {
        GetSwingOrder(0,0,Swingf);
    }
    else if(time<=T0)
    {
        thetime=time-theT0;
        GetSwingOrder(0,1,Swingf);
        theMargins[0]=-theMargins[0];theMargins[1]=-theMargins[1];
        theStep[1]=-theStep[1];
    }
    else
    {
        thetime=0;
    }
    return Design_Skating_Cycle_Based(Body0,Tip0,theMargins,theStep,theT0,thetime,Movemode,Bodytype,Swingtype,duty,Swingf,OutBody,OutTip,outstate);
}

