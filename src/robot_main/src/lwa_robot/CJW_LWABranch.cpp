#include "CJW_LWABranch.h"

void CJW_LWABranch::CJW_Init(void)
{
    LegState.CJW_Init();
    WheelState.CJW_Init();
    ArmState.CJW_Init();
}
/************based on function**********************************************************/
void CJW_LWABranch::SetStruct(struct CJW_BranchStructure* input)
{
    ComStruct=*input;
    for(int i=1;i<ComStruct.BodyN;i++)
    {
        for(int j=0;j<(ComStruct.BodyN-1);j++)
        {
            if(ComStruct.JStruct[i].parent==&(input->JStruct[j]))
            {
                ComStruct.JStruct[i].parent=&(ComStruct.JStruct[j]);
                break;
            }
        }
    }
    for(int model=0;model<BRANCH_MODEL_MAX;model++)
    {
        if(ComStruct.Model_EN[model])
        {
            for(int i=0;i<ComStruct.BModel[model].JointN;i++)
            {
                for(int j=0;j<(ComStruct.BodyN);j++)
                {
                    if(ComStruct.BModel[model].Joint[i]==&(input->Joint[j]))
                    {
                        ComStruct.BModel[model].Joint[i]=&(ComStruct.Joint[j]);
                        break;
                    }
                }
            }
        }
    }
}
void CJW_LWABranch::GetStruct(struct CJW_BranchStructure* output)
{
    *output=ComStruct;
    for(int i=1;i<ComStruct.BodyN;i++)
    {
        for(int j=0;j<(ComStruct.BodyN-1);j++)
        {
            if(output->JStruct[i].parent==&(ComStruct.JStruct[j]))
            {
                output->JStruct[i].parent=&(output->JStruct[j]);
                break;
            }
        }
    }
    for(int model=0;model<BRANCH_MODEL_MAX;model++)
    {
        if(ComStruct.Model_EN[model])
        {
            for(int i=0;i<ComStruct.BModel[model].JointN;i++)
            {
                for(int j=0;j<(ComStruct.BodyN);j++)
                {
                    if(output->BModel[model].Joint[i]==&(ComStruct.Joint[j]))
                    {
                        output->BModel[model].Joint[i]=&(output->Joint[j]);
                        break;
                    }
                }
            }
        }
    }
}
void CJW_LWABranch::GetStructOneJStruct(int jointi,struct CJW_JointStruct *out)
{
    *out=ComStruct.JStruct[jointi];
}
void CJW_LWABranch::ShowStruct(void)
{
    printf("The %s struct is description as:\n",ComStruct.name.c_str());
    for(int i=0;i<ComStruct.BodyN;i++)
    {
        string tempparent="Base";
        if(ComStruct.JStruct[i].parent!=NULL){tempparent=ComStruct.JStruct[i].parent->name;}
        printf("%s-->%s\n",ComStruct.JStruct[i].name.c_str(),tempparent.c_str());
    }
    int themodel=GetModel();
    printf("The branch is now %d model.\n",themodel);
    for(int i=0;i<BRANCH_MODEL_MAX;i++)
    {
        if(ComStruct.Model_EN[i])
        {
            SetModel(i);
            BranchModel->ShowStruct();
        }
    }
    SetModel(themodel);
}
void CJW_LWABranch::ShowBranch(void)
{
    Print_CJW_BranchStructure(&ComStruct);
}
void CJW_LWABranch::SetRootParent(struct CJW_JointStruct* root)
{
    ComStruct.JStruct[0].parent=root;
}
int  CJW_LWABranch::SetModel(int state)
{    
    if(ComStruct.Model_EN[state])
    {
        if(state==BRANCH_LEG_MODEL){BranchModel=&LegState;}
        else if(state==BRANCH_WHEEL_MODEL){BranchModel=&WheelState;}
        else if(state==BRANCH_ARM_MODEL){BranchModel=&ArmState;}
        else{}
        return RIGHT;
    }
    return ERROR;
}
int CJW_LWABranch::GetModel(void)
{
    return BranchModel->GetModel();
}
int CJW_LWABranch::SetState(double state)
{
    BranchModel->SetState(state);
    return RIGHT;
}
double  CJW_LWABranch::GetState(void)
{
    return BranchModel->GetState();
}
int  CJW_LWABranch::SetBodyN(int n)
{
    ComStruct.BodyN=n;
    return RIGHT;
}
int  CJW_LWABranch::GetBodyN(void)
{
    return ComStruct.BodyN;
}
/*********** the branch function *******************************************************/
//get the one joint struct
void CJW_LWABranch::GetOneJStructScrew(int jointi, double screw[6])
{
    for(int ii=0;ii<6;ii++)
    {
        screw[ii]=ComStruct.JStruct[jointi].Screw0[ii];
    }
}
//check the Joint limiation
int CJW_LWABranch::CheckJointLimit(int* error)
{
    int allerror=RIGHT;
    for(int i=0;i<ComStruct.BodyN;i++)
    {
        if(ComStruct.Joint[i].Angle>ComStruct.Joint[i].AngleLimit[1])
        {
            error[i]=JOINT_ERROR_P_UP;allerror=allerror+1;
        }
        else if(ComStruct.Joint[i].Angle<ComStruct.Joint[i].AngleLimit[0])
        {
            error[i]=JOINT_ERROR_P_DOWN;allerror=allerror+1;
        }
        else if(ComStruct.Joint[i].Velocity>ComStruct.Joint[i].VelocityLimit[1])
        {
            error[i]=JOINT_ERROR_V_UP;allerror=allerror+1;
        }
        else if(ComStruct.Joint[i].Velocity<ComStruct.Joint[i].VelocityLimit[0])
        {
            error[i]=JOINT_ERROR_V_DOWN;allerror=allerror+1;
        }
        // else if(ComStruct.Joint[i].Accelation>ComStruct.Joint[i].AccelationLimit[1])
        // {
        //     error[i]=JOINT_ERROR_A_UP;allerror=allerror+1;
        // }
        // else if(ComStruct.Joint[i].Accelation<ComStruct.Joint[i].AccelationLimit[0])
        // {
        //     error[i]=JOINT_ERROR_A_DOWN;allerror=allerror+1;
        // }
        else if(ComStruct.Joint[i].Force>ComStruct.Joint[i].ForceLimit[1])
        {
            error[i]=JOINT_ERROR_F_UP;allerror=allerror+1;
        }
        else if(ComStruct.Joint[i].Force<ComStruct.Joint[i].ForceLimit[0])
        {
            error[i]=JOINT_ERROR_F_DOWN;allerror=allerror+1;
        }
        else
        {
            error[i]=RIGHT;
        }  
    }
    return allerror;
}
//get the all angle state
int CJW_LWABranch::GetALLAngleState(struct CJW_BasedJoint *out)
{
    for(int i=0;i<ComStruct.BodyN;i++)
    {
        out[i]=ComStruct.Joint[i];
    }
    return RIGHT;
}
int CJW_LWABranch::GetALLAngleP(double *out)
{
    for(int i=0;i<ComStruct.BodyN;i++)
    {
        out[i]=ComStruct.Joint[i].Angle;
    }
    return RIGHT;
}
int CJW_LWABranch::GetALLAngleV(double *out)
{
    for(int i=0;i<ComStruct.BodyN;i++)
    {
        out[i]=ComStruct.Joint[i].Velocity;
    }
    return RIGHT;
}
int CJW_LWABranch::GetALLAngleA(double *out)
{
    for(int i=0;i<ComStruct.BodyN;i++)
    {
        out[i]=ComStruct.Joint[i].Accelation;
    }
    return RIGHT;
}
int CJW_LWABranch::GetALLAngleF(double *out)
{
    for(int i=0;i<ComStruct.BodyN;i++)
    {
        out[i]=ComStruct.Joint[i].Force;
    }
    return RIGHT;
}
//get the one angle state
int CJW_LWABranch::GetOneAngleState(int jointi,struct CJW_BasedJoint *out)
{
    out[0]=ComStruct.Joint[jointi];
    return RIGHT;
}
int CJW_LWABranch::GetOneAngleP(int jointi,double *out)
{
    out[0]=ComStruct.Joint[jointi].Angle;
    return RIGHT;
}
int CJW_LWABranch::GetOneAngleV(int jointi,double *out)
{
    out[0]=ComStruct.Joint[jointi].Velocity;
    return RIGHT;
}
int CJW_LWABranch::GetOneAngleA(int jointi,double *out)
{
    out[0]=ComStruct.Joint[jointi].Accelation;
    return RIGHT;
}
int CJW_LWABranch::GetOneAngleF(int jointi,double *out)
{
    out[0]=ComStruct.Joint[jointi].Force;
    return RIGHT;
}
//set the all all state//can not change the limit
int CJW_LWABranch::SetALLAngleState(struct CJW_BasedJoint *out)
{
    for(int i=0;i<ComStruct.BodyN;i++)
    {
        ComStruct.Joint[i].Angle=out[i].Angle;
        ComStruct.Joint[i].Velocity=out[i].Velocity;
        ComStruct.Joint[i].Accelation=out[i].Accelation;
        ComStruct.Joint[i].Force=out[i].Force;
    }
    return RIGHT;
}
int CJW_LWABranch::SetALLAngleP(double *out)
{
    for(int i=0;i<ComStruct.BodyN;i++)
    {
        ComStruct.Joint[i].Angle=out[i];
    }
    return RIGHT;
}
int CJW_LWABranch::SetALLAngleV(double *out)
{
    for(int i=0;i<ComStruct.BodyN;i++)
    {
        ComStruct.Joint[i].Velocity=out[i];
    }
    return RIGHT;
}
int CJW_LWABranch::SetALLAngleA(double *out)
{
    for(int i=0;i<ComStruct.BodyN;i++)
    {
        ComStruct.Joint[i].Accelation=out[i];
    }
    return RIGHT;
}
int CJW_LWABranch::SetALLAngleF(double *out)
{
    for(int i=0;i<ComStruct.BodyN;i++)
    {
        ComStruct.Joint[i].Force=out[i];
    }
    return RIGHT;
}
//set the one state//can not change the limit
int CJW_LWABranch::SetOneAngleState(int jointi,struct CJW_BasedJoint *out)
{
    ComStruct.Joint[jointi].Angle=out[0].Angle;
    ComStruct.Joint[jointi].Velocity=out[0].Velocity;
    ComStruct.Joint[jointi].Accelation=out[0].Accelation;
    ComStruct.Joint[jointi].Force=out[0].Force;
    return RIGHT;
}
int CJW_LWABranch::SetOneAngleP(int jointi,double *out)
{
    ComStruct.Joint[jointi].Angle=out[0];
    return RIGHT;
}
int CJW_LWABranch::SetOneAngleV(int jointi,double *out)
{
    ComStruct.Joint[jointi].Velocity=out[0];
    return RIGHT;
}
int CJW_LWABranch::SetOneAngleA(int jointi,double *out)
{
    ComStruct.Joint[jointi].Accelation=out[0];
    return RIGHT;
}
int CJW_LWABranch::SetOneAngleF(int jointi,double *out)
{
    ComStruct.Joint[jointi].Force=out[0];
    return RIGHT;
}
/*********** the branch modoel function *************************************************/
//get the angle state
int CJW_LWABranch::GetModelAngleState(struct CJW_BasedJoint *out)
{
    return BranchModel->GetAngleState(out);
}
int CJW_LWABranch::GetModelAngleP(double *out)
{
    return BranchModel->GetAngleP(out);
}
int CJW_LWABranch::GetModelAngleV(double *out)
{
    return BranchModel->GetAngleV(out);
}
int CJW_LWABranch::GetModelAngleA(double *out)
{
    return BranchModel->GetAngleA(out);
}
int CJW_LWABranch::GetModelAngleF(double *out)
{
    return BranchModel->GetAngleF(out);
}
//set the banch state
int CJW_LWABranch::SetModelAngleState(struct CJW_BasedJoint *in)
{
    return BranchModel->SetAngleState(in);
}
int CJW_LWABranch::SetModelAngleP(double *in)
{
    return BranchModel->SetAngleP(in);
}
int CJW_LWABranch::SetModelAngleV(double *in)
{
    return BranchModel->SetAngleV(in);
}
int CJW_LWABranch::SetModelAngleA(double *in)
{
    return BranchModel->SetAngleA(in);
}
int CJW_LWABranch::SetModelAngleF(double *in)
{
    return BranchModel->SetAngleF(in);
}
//get the tip state
int CJW_LWABranch::GetModelTip(struct CJW_TipStruct* Tip0)
{
    return BranchModel->GetTip(Tip0);
}
int CJW_LWABranch::GetModelTipG(double TipG[4][4])
{
    return BranchModel->GetTipG(TipG);
}
int CJW_LWABranch::GetModelTipP(double TipP[3])
{
    return BranchModel->GetTipP(TipP);
}
int CJW_LWABranch::GetModelTipV(double TipV[6])
{
    return BranchModel->GetTipV(TipV);
}
int CJW_LWABranch::GetModelTipA(double TipA[6])
{
    return BranchModel->GetTipA(TipA);
}
int CJW_LWABranch::GetModelTipF(double TipF[6])
{
    return BranchModel->GetTipF(TipF);
}
//set the tip state
int CJW_LWABranch::SetModelTip(struct CJW_TipStruct* Tip0)
{
    return BranchModel->SetTip(Tip0);
}
int CJW_LWABranch::SetModelTipG(double TipG[4][4])
{
    return BranchModel->SetTipG(TipG);
}
int CJW_LWABranch::SetModelTipP(double TipP[3])
{
    return BranchModel->SetTipP(TipP);
}
int CJW_LWABranch::SetModelTipV(double TipV[6])
{
    return BranchModel->SetTipV(TipV);
}
int CJW_LWABranch::SetModelTipA(double TipA[6])
{
    return BranchModel->SetTipA(TipA);
}
int CJW_LWABranch::SetModelTipF(double TipF[6])
{
    return BranchModel->SetTipF(TipF);
}
//trans between the angle and the tip///the neccesarty for the new type branch
int CJW_LWABranch::ModelAngleToAllBodyStateRoot(void)
{
    return BranchModel->AngleToAllBodyStateRoot();
}
struct CJW_BranchDynamicState  CJW_LWABranch::ModelGetDynamicStateRoot(void)
{
    return BranchModel->GetDynamicStateRoot();
}//based on AngleToAllBodyStateRoot();
int CJW_LWABranch::ModelGetJacobianTip(double *outJ)
{
    return BranchModel->GetJacobianTip(outJ);
}//based on AngleToAllBodyStateRoot();
int CJW_LWABranch::ModelGetGravityComForJoint(double* bodyR,double *JointF)
{
    return BranchModel->GetGravityComForJoint(bodyR,JointF);
}//based on AngleToAllBodyStateRoot();
int CJW_LWABranch::ModelAnglePTransToTipG(void)
{
    return BranchModel->AnglePTransToTipG();
}//based on AngleToAllBodyStateRoot();
int CJW_LWABranch::ModelAngleVTransToTipV(void)
{
    return BranchModel->AngleVTransToTipV();
}//based on AngleToAllBodyStateRoot();
int CJW_LWABranch::ModelAngleFTransToTipF(void)
{
    return BranchModel->AngleFTransToTipF();
}//based on AngleToAllBodyStateRoot();
int CJW_LWABranch::ModelTipGTransToAngleP(double BodyG[4][4])
{
    return BranchModel->TipGTransToAngleP(BodyG);
}
int CJW_LWABranch::ModelTipVTransToAngleV(void)
{
    return BranchModel->TipVTransToAngleV();
}
int CJW_LWABranch::ModelTipFTransToAngleF(void)
{
    return BranchModel->TipFTransToAngleF();
}//based on AngleToAllBodyStateRoot();
