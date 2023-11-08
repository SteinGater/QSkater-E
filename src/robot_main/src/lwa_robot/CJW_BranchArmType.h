#ifndef CJW_BRANCHARMTYPE
#define CJW_BRANCHARMTYPE

#include "CJW_BranchBased.h"

/***************************定义常量************************/
//define branch state
#define BRANCH_ARM_MODEL           2


/***********************************************define model function**********************************************/

/********************************************************
the robot branch is the arm model
*********************************************************/
class CJW_BranchArmType: public CJW_BranchBased
{
public:
    /*************Constructor****************************************************************/
    CJW_BranchArmType(struct CJW_BranchModel* Input):CJW_BranchBased(Input){}
    ~CJW_BranchArmType(void){}
    virtual void CJW_Init(void);
    /************special on function**********************************************************/
    virtual int  TipGTransToAngleP(double BodyG[4][4]);
    virtual int  TipVTransToAngleV(void);
    virtual int  AngleFTransToTipF(void);

protected:

private:

};


#endif
