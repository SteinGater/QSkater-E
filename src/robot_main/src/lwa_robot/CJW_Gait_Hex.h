#ifndef CJW_GAIT_HEX
#define CJW_GAIT_HEX

#include "CJW_GaitBased.h"


/*************************************************/
//define the leg num mapping
#define HEXAPOD_NUM               6
//define the gait of hexapod
#define HEXAPOD_GAIT_Tri          0
#define HEXAPOD_GAIT_Four         1
#define HEXAPOD_GAIT_Five         2



/***********************************************define hexapod gait**********************************************/
/********************************************************
    the hexapodrobot gait class has:
    3+3 line gait
    3+3 circle gait
    4+2 line gait
    4+2 circle gait
    5+1 line gait
    5+1 circle gait
    *********************************************************/
class CJW_Gait_Hex:public CJW_GaitBased
{
public:
    /*************Constructor****************************************************************/
    CJW_Gait_Hex(void)
    {
        GaitN=HEXAPOD_NUM;
        for(int legi=0;legi<GaitN;legi++)
        {
            GaitBranchModel[legi]=BRANCH_LEG_MODEL;
        }
    }
    ~CJW_Gait_Hex(void){}
    virtual int  GetSwingOrder(int gaittype,int num,int* out);
    virtual int  GetWheelDirection(int gaittype,double* dir);
    /*****************************************type gait**************************************/
    virtual int Design_Leg_Position0(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0, double dir[3], double StepH,double T0,double time,
    int Movemode,int Bodytype,int GaitType,int Swingtype,
    struct CJW_JointStruct* OutBody,struct CJW_TipStruct* OutTip,double* outstate);
    /****************************virtual gait for typical position arm design***********************************************/
    virtual int Design_Arm_Position0(int* arm_flag,struct CJW_TipStruct* Start, struct CJW_TipStruct* End,double T0,double time,int Movemode,
                                        struct CJW_TipStruct* OutTip,double* outstate);
    /****************************virtual gait for typical position wheel design***********************************************/
    virtual int Design_Wheel_Position0(struct CJW_JointStruct* BodyStart, struct CJW_JointStruct* BodyEnd,double T0,double time,int Movemode,
                                        struct CJW_JointStruct* OutTip,double* outstate);
    virtual int Design_Skating_Position0(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0,double Margins[2],double Step[2],double T0,double duty,double time,
                                    int Movemode,int Bodytype,int GaitType,int Swingtype,
                                    struct CJW_JointStruct* OutBody,struct CJW_TipStruct* OutTip,double* outstate);

protected:

private:

};

#endif
