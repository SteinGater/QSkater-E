#ifndef CJW_GAIT_QUA
#define CJW_GAIT_QUA

#include "CJW_GaitBased.h"


/*************************************************/
//define the leg num mapping
#define QUADRUPED_NUM             4
//define the gait of quadruped
#define QUADRUPED_GAIT_TROT          0
#define QUADRUPED_GAIT_PACE          1
#define QUADRUPED_GAIT_BOUNDING      2
#define QUADRUPED_GAIT_STATIC        10
#define QUADRUPED_GAIT_SWIZZLING     11

/*****************************************quadruped gait*******************************************************/
/********************************************************
    the quadrued gait class has:
    trot gait
    pace gait
    *********************************************************/
class CJW_Gait_Qua:public CJW_GaitBased
{
public:
    /*************Constructor****************************************************************/
    CJW_Gait_Qua(void)
    {
        GaitN=QUADRUPED_NUM;
        for(int legi=0;legi<GaitN;legi++)
        {
            GaitBranchModel[legi]=BRANCH_LEG_MODEL;
        }
    }
    ~CJW_Gait_Qua(void){}
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
