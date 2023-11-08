#include "CJW_BranchArmType.h"






/**************************************achieve the function of class*******************************************/

/*************Constructor****************************************************************/
void CJW_BranchArmType::CJW_Init(void)
{
    TheStructure->modelstate=BRANCH_ARM_MODEL;
}
/************special on function**********************************************************/
int  CJW_BranchArmType::AngleFTransToTipF(void)
{
    return ERROR;
}
int  CJW_BranchArmType::TipGTransToAngleP(double BodyG[4][4])
{
    return ERROR;
}
int  CJW_BranchArmType::TipVTransToAngleV(void)
{
    return ERROR;
}


