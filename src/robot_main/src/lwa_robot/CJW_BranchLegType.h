#ifndef CJW_BRANCHLEGTYPE
#define CJW_BRANCHLEGTYPE

#include "CJW_BranchBased.h"

//define branch state
#define BRANCH_LEG_MODEL           0

/***********************************************define model function**********************************************/
/********************************************************
the robot branch is the type structure (3 DoF)
the rorate angle is Z-Y-Y and X-axis is distance
the initial constructure is line along X-axis
*********************************************************/
class CJW_BranchLegType: public CJW_BranchBased
{
public:
    /*************Constructor****************************************************************/
    CJW_BranchLegType(struct CJW_BranchModel* Input):CJW_BranchBased(Input){}
    ~CJW_BranchLegType(void){}
    virtual void CJW_Init(void);
    /************special on function**********************************************************/
    //because the inversense only use the position after get angle will compute the orientation of tip
    virtual int  TipGTransToAngleP(double BodyG[4][4]);
    //because the inversense only use the linear velocity after get angle will compute the rorate velocity of tip
    virtual int  TipVTransToAngleV(void);
    virtual int  AngleFTransToTipF(void);
    /************reset for compute special**********************************************************/



protected:
    int JN;
    double LengthL[3];
    double LY;


private:

};

#endif
