#include "CJW_Gait_Hex.h"



/*******************************************************************************/
int  CJW_Gait_Hex::GetSwingOrder(int gaittype,int num,int* out)
{
    out[0]=DESIGN_SUP_STATE;out[2]=DESIGN_SUP_STATE;out[4]=DESIGN_SUP_STATE;
    out[1]=DESIGN_SUP_STATE;out[3]=DESIGN_SUP_STATE;out[5]=DESIGN_SUP_STATE;
    if(gaittype==HEXAPOD_GAIT_Four)
    {
        int then=num%3;
        out[then]=DESIGN_SWI_STATE;out[then+3]=DESIGN_SWI_STATE;
    }
    else if(gaittype==HEXAPOD_GAIT_Five)
    {
        out[num]=DESIGN_SWI_STATE;
    }
    else//trot
    {
        int then=num%2;
        out[then]=DESIGN_SWI_STATE;out[then+2]=DESIGN_SWI_STATE;out[then+4]=DESIGN_SWI_STATE;
    }
    return RIGHT;
}
int CJW_Gait_Hex::GetWheelDirection(int gaittype,double* dir)
{
    dir[0]=1;dir[1]=1;dir[2]=1;
    dir[3]=-1;dir[4]=-1;dir[5]=-1;
    return RIGHT;
}
/*****************************************type gait**************************************/
int CJW_Gait_Hex::Design_Leg_Position0(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0, double dir[3], double StepH,double T0,double time,
int Movemode,int Bodytype,int GaitType,int Swingtype,
struct CJW_JointStruct* OutBody,struct CJW_TipStruct* OutTip,double* outstate)
{
    double SwingStart0[HEXAPOD_NUM]={0,0.5,0,0.5,0,0.5};
    double SwingDuty0[HEXAPOD_NUM]={0.5,0.5,0.5,0.5,0.5,0.5};
    if(GaitType==HEXAPOD_GAIT_Four)
    {
        SwingStart0[0]=0;               SwingStart0[1]=1.0/3;           SwingStart0[2]=2.0/3;
        SwingStart0[3]=SwingStart0[0];  SwingStart0[4]=SwingStart0[1];  SwingStart0[5]=SwingStart0[2];
        SwingDuty0[0]=1.0/3;        SwingDuty0[1]=SwingDuty0[0];SwingDuty0[2]=SwingDuty0[0];
        SwingDuty0[3]=SwingDuty0[0];SwingDuty0[4]=SwingDuty0[0];SwingDuty0[5]=SwingDuty0[0];
    }
    else if(GaitType==HEXAPOD_GAIT_Five)
    {
        SwingStart0[0]=0;       SwingStart0[1]=1.0/6;   SwingStart0[2]=2.0/6;
        SwingStart0[3]=3.0/6;   SwingStart0[4]=4.0/6;   SwingStart0[5]=5.0/6;
        SwingDuty0[0]=1.0/6;        SwingDuty0[1]=SwingDuty0[0];SwingDuty0[2]=SwingDuty0[0];
        SwingDuty0[3]=SwingDuty0[0];SwingDuty0[4]=SwingDuty0[0];SwingDuty0[5]=SwingDuty0[0];
    }
    else//HEXAPOD_GAIT_Tri
    {

    }
    return Design_Gait_Cycle_Based(Body0,Tip0,dir,StepH,T0,time,Movemode,Bodytype,Swingtype,SwingStart0,SwingDuty0,OutBody,OutTip,outstate);
}
/****************************virtual gait for typical position arm design***********************************************/
int CJW_Gait_Hex::Design_Arm_Position0(int* arm_flag,struct CJW_TipStruct* Start, struct CJW_TipStruct* End,double T0,double time,int Movemode,
                                    struct CJW_TipStruct* OutTip,double* outstate)
{
    return ERROR;
}

/****************************virtual gait for typical position wheel design***********************************************/
int CJW_Gait_Hex::Design_Wheel_Position0(struct CJW_JointStruct* BodyStart, struct CJW_JointStruct* BodyEnd,double T0,double time,int Movemode,
                                    struct CJW_JointStruct* OutTip,double* outstate)
{
    return ERROR;
}

int CJW_Gait_Hex::Design_Skating_Position0(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0,double Margins[2],double Step[2],double T0,double duty,double time,
int Movemode,int Bodytype,int GaitType,int Swingtype,
struct CJW_JointStruct* OutBody,struct CJW_TipStruct* OutTip,double* outstate)
{
    //double SwingT0=0.5;
    int Swingf[HEXAPOD_NUM]={0,1,0,1,0,1};
    double thetime=time;double theT0=T0/2;
    double theMargins[2]={-fabs(Margins[0]),fabs(Margins[1])};
    double theStep[2]={fabs(Step[0]),fabs(Step[1])};
    if(time<=0)
    {
        thetime=0;
    }
    else if(time<=theT0)
    {
        GetSwingOrder(GaitType,0,Swingf);
    }
    else if(time<=T0)
    {
        thetime=time-theT0;
        GetSwingOrder(GaitType,1,Swingf);
        theMargins[0]=-theMargins[0];theMargins[1]=-theMargins[1];
        theStep[1]=-theStep[1];
    }
    else
    {
        thetime=0;
    }
    return Design_Skating_Cycle_Based(Body0,Tip0,theMargins,theStep,theT0,thetime,Movemode,Bodytype,Swingtype,duty,Swingf,OutBody,OutTip,outstate);
}
