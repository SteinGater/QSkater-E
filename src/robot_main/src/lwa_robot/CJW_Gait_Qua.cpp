#include "CJW_Gait_Qua.h"


/*******************************************************************************/
int  CJW_Gait_Qua::GetSwingOrder(int gaittype,int num,int* out)
{
    out[0]=DESIGN_SUP_STATE;out[1]=DESIGN_SUP_STATE;out[2]=DESIGN_SUP_STATE;out[3]=DESIGN_SUP_STATE;
    if(gaittype==QUADRUPED_GAIT_TROT)
    {
        switch(num%2)
        {
            case 0:out[1]=DESIGN_SWI_STATE;out[3]=DESIGN_SWI_STATE;break;
            case 1:out[0]=DESIGN_SWI_STATE;out[2]=DESIGN_SWI_STATE;break;
            default: return ERROR;
        }
    }
    else if(gaittype==QUADRUPED_GAIT_PACE)
    {
        switch(num%2)
        {
            case 0:out[1]=DESIGN_SWI_STATE;out[2]=DESIGN_SWI_STATE;break;
            case 1:out[0]=DESIGN_SWI_STATE;out[3]=DESIGN_SWI_STATE;break;
            default: return ERROR;
        }
    }
    else if(gaittype==QUADRUPED_GAIT_BOUNDING)
    {
        switch(num%2)
        {
            case 0:out[0]=DESIGN_SWI_STATE;out[1]=DESIGN_SWI_STATE;break;
            case 1:out[2]=DESIGN_SWI_STATE;out[3]=DESIGN_SWI_STATE;break;
            default: return ERROR;
        }
    }
    else if(gaittype==QUADRUPED_GAIT_STATIC)
    {
        switch(num%4)
        {
            case 0:out[0]=DESIGN_SWI_STATE;break;
            case 1:out[1]=DESIGN_SWI_STATE;break;
            case 2:out[2]=DESIGN_SWI_STATE;break;
            case 3:out[3]=DESIGN_SWI_STATE;break;
            default: return ERROR;
        }
    }
    else
    {

    }
    return RIGHT;
}
int CJW_Gait_Qua::GetWheelDirection(int gaittype,double* dir)
{
    if(gaittype==QUADRUPED_GAIT_TROT)
    {
        dir[0]=-1;dir[1]=1;dir[2]=-1;dir[3]=1;
    }
    else if(gaittype==QUADRUPED_GAIT_PACE)
    {
        dir[0]=-1;dir[1]=1;dir[2]=1;dir[3]=-1;
    }
    else if(gaittype==QUADRUPED_GAIT_STATIC)
    {
        dir[0]=-1;dir[1]=1;dir[2]=1;dir[3]=-1;
    }
    else
    {
        dir[0]=0;dir[1]=0;dir[2]=-0;dir[3]=0;
    }
    return RIGHT;
}
/*****************************************type gait**************************************/
int CJW_Gait_Qua::Design_Leg_Position0(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0, double dir[3], double StepH,double T0,double time,
int Movemode,int Bodytype,int GaitType,int Swingtype,
struct CJW_JointStruct* OutBody,struct CJW_TipStruct* OutTip,double* outstate)
{
    double SwingStart0[QUADRUPED_NUM]={0,0.5,0,0.5};
    double SwingDuty0[QUADRUPED_NUM]={0.5,0.5,0.5,0.5};
    if(GaitType==QUADRUPED_GAIT_PACE)
    {
        SwingStart0[0]=0;SwingStart0[1]=0.5;SwingStart0[2]=0.5;SwingStart0[3]=0;
    }
    else//QUADRUPED_GAIT_TROT
    {

    }
    return Design_Gait_Cycle_Based(Body0,Tip0,dir,StepH,T0,time,Movemode,Bodytype,Swingtype,SwingStart0,SwingDuty0,OutBody,OutTip,outstate);
}
/****************************virtual gait for typical position arm design***********************************************/
int CJW_Gait_Qua::Design_Arm_Position0(int* arm_flag,struct CJW_TipStruct* Start, struct CJW_TipStruct* End,double T0,double time,int Movemode,
                                    struct CJW_TipStruct* OutTip,double* outstate)
{
    return ERROR;
}

/****************************virtual gait for typical position wheel design***********************************************/
int CJW_Gait_Qua::Design_Wheel_Position0(struct CJW_JointStruct* BodyStart, struct CJW_JointStruct* BodyEnd,double T0,double time,int Movemode,
                                    struct CJW_JointStruct* OutTip,double* outstate)
{
    return ERROR;
}

int CJW_Gait_Qua::Design_Skating_Position0(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0,double Margins[2],double Step[2],double T0,double duty,double time,
int Movemode,int Bodytype,int GaitType,int Swingtype,
struct CJW_JointStruct* OutBody,struct CJW_TipStruct* OutTip,double* outstate)
{
    //double SwingT0=0.5;
    int Swingf[QUADRUPED_NUM]={0,1,0,1};
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
