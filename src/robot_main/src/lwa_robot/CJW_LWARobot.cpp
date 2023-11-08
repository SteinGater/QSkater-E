
#include "CJW_LWARobot.h"


void CJW_LWARobot::CJW_Init(void)
{
    for(int i=0;i<BranchN;i++)
    {
        BranchS[i].CJW_Init();
    }
    PV_KF_Estimator.SetType(EKF_IL);
}

/************based on function**********************************************************/
int CJW_LWARobot::SetStruct(struct CJW_RobotStructure* input)
{
    BranchN=input->BranchN;
    MainBody=input->MainBody;
    MainBody.parent=NULL;
    for(int i=0;i<BranchN;i++)
    {
        BranchP[i]=input->BranchP[i];
        BranchP[i].parent=&(MainBody);
        BranchS[i].SetStruct(&(input->BranchBody[i]));
        BranchS[i].SetRootParent(&(BranchP[i]));
    }
    return RIGHT;
}
int CJW_LWARobot::GetStruct(struct CJW_RobotStructure* output)
{
    output->BranchN=BranchN;
    output->MainBody=MainBody;
    output->MainBody.parent=NULL;
    for(int i=0;i<BranchN;i++)
    {
        output->BranchP[i]=BranchP[i];
        BranchS[i].GetStruct(&(output->BranchBody[i]));
        output->BranchP[i].parent=&(output->MainBody);
        output->BranchBody[i].JStruct[0].parent=&(output->BranchP[i]);
    }
    return RIGHT;
}
void CJW_LWARobot::ShowStruct(void)
{
    printf("the robot %s: BranchN is %d\n; \n",robotname.c_str(),BranchN);
    for(int i=0;i<BranchN;i++)
    {
        printf("/*********************************************************/\n");
        printf("Branch No.%d struct is \n",i);
        printf("%s-->%s\n",BranchP[i].name.c_str(),BranchP[i].parent->name.c_str());
        BranchS[i].ShowStruct();
    }
}
void CJW_LWARobot::ShowMainBody(void)
{
    printf("the main body %s is \n",MainBody.name.c_str());
    Print_CJW_JointStruct(&MainBody);
}
void CJW_LWARobot::ShowBranch(void)
{
    printf("the robot %s: BranchN is %d\n; \n",robotname.c_str(),BranchN);
    Print_CJW_JointStruct(&(MainBody));
    for(int i=0;i<BranchN;i++)
    {
        printf("/*********************************************************/\n");
        printf("Branch No.%d struct is \n",i);
        Print_CJW_JointStruct(&(BranchP[i]));
        BranchS[i].ShowBranch();
    }
}
void CJW_LWARobot::ShowALLModelAngle(void)
{
    printf("the robot %s state is \n",robotname.c_str());
    printf("angleP is \n");
    double temp[BRANCH_BODY_MAX];
    for(int legi=0;legi<BranchN;legi++)
    {
        BranchS[legi].GetALLAngleP(temp);
        for(int i=0;i<BranchS[legi].GetBodyN();i++)
        {
            printf("%.3f \t",temp[i]);
        }
        printf("\n");
    }
    printf("angleV is \n");
    for(int legi=0;legi<BranchN;legi++)
    {
        BranchS[legi].GetALLAngleV(temp);
        for(int i=0;i<BranchS[legi].GetBodyN();i++)
        {
            printf("%.3f \t",temp[i]);
        }
        printf("\n");
    }
    printf("angleF is \n");
    for(int legi=0;legi<BranchN;legi++)
    {
        BranchS[legi].GetALLAngleF(temp);
        for(int i=0;i<BranchS[legi].GetBodyN();i++)
        {
            printf("%.3f \t",temp[i]);
        }
        printf("\n");
    }
}
void CJW_LWARobot::ShowALLTipAbsolute(void)
{
    printf("the tipe of robot %s state is \n",robotname.c_str());
    for(int legi=0;legi<BranchN;legi++)
    {
        printf("tip %d is \n",legi);
        Print_CJW_TipStruct(&(TipAbsolute[legi]));
    }
}
/***********branch normal function *************************************************/
int CJW_LWARobot::GetBranchN(void)
{
    return BranchN;
}
int CJW_LWARobot::SetRobotModel(int model)
{
    RobotModel=model;
    return RIGHT;
}
int CJW_LWARobot::GetRobotModel(void)
{
    return RobotModel;
}
int CJW_LWARobot::SetBranchiModel(int legi,int state)
{
    return BranchS[legi].SetModel(state);
}
int CJW_LWARobot::GetBranchiModel(int legi)
{
    return BranchS[legi].GetModel();
}
int CJW_LWARobot::SetBranchiState(int legi,double state)
{
    return BranchS[legi].SetState(state);
}
double CJW_LWARobot::GetBranchiState(int legi)
{
    return BranchS[legi].GetState();
}
int CJW_LWARobot::SetBranchModel(int* state)
{
    for(int legi=0;legi<BranchN;legi++)
    {
        BranchS[legi].SetModel(state[legi]);
    }
    return RIGHT;
}
int CJW_LWARobot::GetBranchModel(int* state)
{
    for(int legi=0;legi<BranchN;legi++)
    {
        state[legi]=BranchS[legi].GetModel();
    }
    return RIGHT;
}
int CJW_LWARobot::SetBranchState(double* state)
{
    for(int legi=0;legi<BranchN;legi++)
    {
        BranchS[legi].SetState(state[legi]);
    }
    return RIGHT;
}
int CJW_LWARobot::GetBranchState(double* state)
{
    for(int legi=0;legi<BranchN;legi++)
    {
        state[legi]=BranchS[legi].GetState();
    }
    return RIGHT;
}
//get the angle state
int CJW_LWARobot::CheckBranchiLimit(int legi,int* error)
{
    return BranchS[legi].CheckJointLimit(error);
}
int CJW_LWARobot::GetBranchiAngleState(int legi,struct CJW_BasedJoint *out)
{
    BranchS[legi].GetALLAngleState(out);
    return RIGHT;
}
int CJW_LWARobot::GetBranchiAngleP(int legi,double *out)
{
    BranchS[legi].GetALLAngleP(out);
    return RIGHT;
}
int CJW_LWARobot::GetBranchiAngleV(int legi,double *out)
{
    BranchS[legi].GetALLAngleV(out);
    return RIGHT;
}
int CJW_LWARobot::GetBranchiAngleA(int legi,double *out)
{
    BranchS[legi].GetALLAngleA(out);
    return RIGHT;
}
int CJW_LWARobot::GetBranchiAngleF(int legi,double *out)
{
    BranchS[legi].GetALLAngleF(out);
    return RIGHT;
}
int CJW_LWARobot::GetBranchiModelAngleState(int legi,struct CJW_BasedJoint *out)
{
    BranchS[legi].GetModelAngleState(out);
    return RIGHT;
}
int CJW_LWARobot::GetBranchiModelAngleP(int legi,double *out)
{
    BranchS[legi].GetModelAngleP(out);
    return RIGHT;
}
int CJW_LWARobot::GetBranchiModelAngleV(int legi,double *out)
{
    BranchS[legi].GetModelAngleV(out);
    return RIGHT;
}
int CJW_LWARobot::GetBranchiModelAngleA(int legi,double *out)
{
    BranchS[legi].GetModelAngleA(out);
    return RIGHT;
}
int CJW_LWARobot::GetBranchiModelAngleF(int legi,double *out)
{
    BranchS[legi].GetModelAngleF(out);
    return RIGHT;
}
//set the banch state
int CJW_LWARobot::SetBranchiAngleState(int legi,struct CJW_BasedJoint *in){BranchS[legi].SetALLAngleState(in);return RIGHT;}
int CJW_LWARobot::SetBranchiAngleP(int legi,double *in)
{
    BranchS[legi].SetALLAngleP(in);
    return RIGHT;
}
int CJW_LWARobot::SetBranchiAngleV(int legi,double *in)
{
    BranchS[legi].SetALLAngleV(in);
    return RIGHT;
}
int CJW_LWARobot::SetBranchiAngleA(int legi,double *in)
{
    BranchS[legi].SetALLAngleA(in);
    return RIGHT;
}
int CJW_LWARobot::SetBranchiAngleF(int legi,double *in)
{
    BranchS[legi].SetALLAngleF(in);
    return RIGHT;
}
int CJW_LWARobot::SetBranchiModelAngleState(int legi,struct CJW_BasedJoint *in)
{
    BranchS[legi].SetModelAngleState(in);
    return RIGHT;
}
int CJW_LWARobot::SetBranchiModelAngleP(int legi,double *in)
{
    BranchS[legi].SetModelAngleP(in);
    return RIGHT;
}
int CJW_LWARobot::SetBranchiModelAngleV(int legi,double *in)
{
    BranchS[legi].SetModelAngleV(in);
    return RIGHT;
}
int CJW_LWARobot::SetBranchiModelAngleA(int legi,double *in)
{
    BranchS[legi].SetModelAngleA(in);
    return RIGHT;
}
int CJW_LWARobot::SetBranchiModelAngleF(int legi,double *in)
{
    BranchS[legi].SetModelAngleF(in);
    return RIGHT;
}
//get and get the tip state
int CJW_LWARobot::GetBranchiModelTip(int legi,CJW_TipStruct* Tip0)
{
    BranchS[legi].GetModelTip(Tip0);
    return RIGHT;
}
int CJW_LWARobot::GetBranchiModelTipG(int legi,double TipG[4][4])
{
    BranchS[legi].GetModelTipG(TipG);
    return RIGHT;
}
int CJW_LWARobot::GetBranchiModelTipV(int legi,double TipV[6])
{
    BranchS[legi].GetModelTipV(TipV);
    return RIGHT;
}
int CJW_LWARobot::GetBranchiModelTipA(int legi,double TipA[6])
{
    BranchS[legi].GetModelTipA(TipA);
    return RIGHT;
}
int CJW_LWARobot::GetBranchiModelTipF(int legi,double TipF[6])
{
    BranchS[legi].GetModelTipF(TipF);
    return RIGHT;
}
int CJW_LWARobot::SetBranchiModelTip(int legi,CJW_TipStruct* Tip0)
{
    BranchS[legi].SetModelTip(Tip0);
    return RIGHT;
}
int CJW_LWARobot::SetBranchiModelTipG(int legi,double TipG[4][4])
{
    BranchS[legi].SetModelTipG(TipG);
    return RIGHT;
}
int CJW_LWARobot::SetBranchiModelTipV(int legi,double TipV[6])
{
    BranchS[legi].SetModelTipV(TipV);
    return RIGHT;
}
int CJW_LWARobot::SetBranchiModelTipA(int legi,double TipA[6])
{
    BranchS[legi].SetModelTipA(TipA);
    return RIGHT;
}
int CJW_LWARobot::SetBranchiModelTipF(int legi,double TipF[6])
{
    BranchS[legi].SetModelTipF(TipF);
    return RIGHT;
}
//trans between the angle and the tip
int CJW_LWARobot::BranchiModelAngleToAllBodyStateRoot(int legi)
{
    return BranchS[legi].ModelAngleToAllBodyStateRoot();
}
struct CJW_BranchDynamicState CJW_LWARobot::BranchiModelGetDynamicStateRoot(int legi)
{
    return BranchS[legi].ModelGetDynamicStateRoot();
}
int CJW_LWARobot::BranchiModelGetJacobianTip(int legi,double *outJ)
{
    return BranchS[legi].ModelGetJacobianTip(outJ);
}
int CJW_LWARobot::BranchiModelGetGravityComForJoint(int legi,double *JointF)
{
    double mybodyR[9]={MainBody.G0[0][0],MainBody.G0[0][1],MainBody.G0[0][2],
    MainBody.G0[1][0],MainBody.G0[1][1],MainBody.G0[1][2],
    MainBody.G0[2][1],MainBody.G0[2][1],MainBody.G0[2][2]};
    return BranchS[legi].ModelGetGravityComForJoint(mybodyR,JointF);
}
int CJW_LWARobot::BranchiModelAnglePTransToTipG(int legi)
{
    return BranchS[legi].ModelAnglePTransToTipG();
}
int CJW_LWARobot::BranchiModelAngleVTransToTipV(int legi)
{
    return BranchS[legi].ModelAngleVTransToTipV();
}
int CJW_LWARobot::BranchiModelAngleFTransToTipF(int legi)
{
    return BranchS[legi].ModelAngleFTransToTipF();
}
int CJW_LWARobot::BranchiModelTipGTransToAngleP(int legi)
{
    return BranchS[legi].ModelTipGTransToAngleP(MainBody.G0);
}
int CJW_LWARobot::BranchiModelTipVTransToAngleV(int legi)
{
    return BranchS[legi].ModelTipVTransToAngleV();
}
int CJW_LWARobot::BranchiModelTipFTransToAngleF(int legi)
{
    return BranchS[legi].ModelTipFTransToAngleF();
}

/***********mainbody normal function**********************************************************************************/
int CJW_LWARobot::GetMainBodyState(struct CJW_JointStruct *out)
{
    out[0]=MainBody;
    return RIGHT;
}
int CJW_LWARobot::SetMainBodyState(struct CJW_JointStruct *in)
{
    SetMainBodyG(in->G0);
    SetMainBodyV(in->Body.BodyV);
    SetMainBodyA(in->Body.BodyA);
    SetMainBodyF(in->Body.BodyF);
    return RIGHT;
}
int CJW_LWARobot::GetMainBodyR(double BodyR[3][3])
{
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            BodyR[i][j]=MainBody.G0[i][j];
        }
    }
    return RIGHT;
}
int CJW_LWARobot::GetMainBodyP(double BodyP[3])
{
    for(int i=0;i<3;i++)
    {
        BodyP[i]=MainBody.G0[i][3];
    }
    return RIGHT;
}
int CJW_LWARobot::GetMainBodyG(double BodyG[4][4])
{
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            BodyG[i][j]=MainBody.G0[i][j];
        }
    }
    return RIGHT;
}
int CJW_LWARobot::GetMainBodyV(double BodyV[6])
{
    for(int i=0;i<6;i++)
    {
        BodyV[i]=MainBody.Body.BodyV[i];
    }
    return RIGHT;
}
int CJW_LWARobot::GetMainBodyw(double Bodyw[3])
{
    for(int i=0;i<3;i++)
    {
        Bodyw[i]=MainBody.Body.BodyV[i];
    }
    return RIGHT;
}
int CJW_LWARobot::GetMainBodyv(double Bodyv[3])
{
    for(int i=0;i<3;i++)
    {
        Bodyv[i]=MainBody.Body.BodyV[i+3];
    }
    return RIGHT;
}
int CJW_LWARobot::GetMainBodyA(double BodyA[6])
{
    for(int i=0;i<6;i++)
    {
        BodyA[i]=MainBody.Body.BodyA[i];
    }
    return RIGHT;
}
int CJW_LWARobot::GetMainBodyAw(double BodyAw[3])
{
    for(int i=0;i<3;i++)
    {
        BodyAw[i]=MainBody.Body.BodyA[i];
    }
    return RIGHT;
}
int CJW_LWARobot::GetMainBodyAv(double BodyAv[3])
{
    for(int i=0;i<3;i++)
    {
        BodyAv[i]=MainBody.Body.BodyA[i+3];
    }
    return RIGHT;
}
int CJW_LWARobot::GetMainBodyF(double BodyF[6])
{
    for(int i=0;i<6;i++)
    {
        BodyF[i]=MainBody.Body.BodyF[i];
    }
    return RIGHT;
}
int CJW_LWARobot::GetMainBodyFf(double BodyFf[3])
{
    for(int i=0;i<3;i++)
    {
        BodyFf[i]=MainBody.Body.BodyF[i+3];
    }
    return RIGHT;
}
int CJW_LWARobot::GetMainBodyFt(double BodyFt[3])
{
    for(int i=0;i<3;i++)
    {
        BodyFt[i]=MainBody.Body.BodyF[i];
    }
    return RIGHT;
}
int CJW_LWARobot::SetMainBodyR(double BodyR[3][3])
{
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            MainBody.G0[i][j]=BodyR[i][j];
        }
    }
    return RIGHT;
}
int CJW_LWARobot::SetMainBodyP(double BodyP[3])
{
    for(int i=0;i<3;i++)
    {
         MainBody.G0[i][3]=BodyP[i];
    }
    return RIGHT;
}
int CJW_LWARobot::SetMainBodyG(double BodyG[4][4])
{
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            MainBody.G0[i][j]=BodyG[i][j];
        }
    }
    return RIGHT;
}
int CJW_LWARobot::SetMainBodyV(double BodyV[6])
{
    for(int i=0;i<6;i++)
    {
        MainBody.Body.BodyV[i]=BodyV[i];
    }
    return RIGHT;
}
int CJW_LWARobot::SetMainBodyw(double Bodyw[3])
{
    for(int i=0;i<3;i++)
    {
        MainBody.Body.BodyV[i]=Bodyw[i];
    }
    return RIGHT;
}
int CJW_LWARobot::SetMainBodyv(double Bodyv[3])
{
    for(int i=0;i<3;i++)
    {
        MainBody.Body.BodyV[i+3]=Bodyv[i];
    }
    return RIGHT;
}
int CJW_LWARobot::SetMainBodyA(double BodyA[6])
{
    for(int i=0;i<6;i++)
    {
        MainBody.Body.BodyA[i]=BodyA[i];
    }
    return RIGHT;
}
int CJW_LWARobot::SetMainBodyAw(double BodyAw[3])
{
    for(int i=0;i<3;i++)
    {
        MainBody.Body.BodyA[i]=BodyAw[i];
    }
    return RIGHT;
}
int CJW_LWARobot::SetMainBodyAv(double BodyAv[3])
{
    for(int i=0;i<3;i++)
    {
        MainBody.Body.BodyA[i+3]=BodyAv[i];
    }
    return RIGHT;
}
int CJW_LWARobot::SetMainBodyF(double BodyF[6])
{
    for(int i=0;i<6;i++)
    {
        MainBody.Body.BodyF[i]=BodyF[i];
    }
    return RIGHT;
}
int CJW_LWARobot::SetMainBodyFf(double BodyFf[3])
{
    for(int i=0;i<3;i++)
    {
        MainBody.Body.BodyF[i+3]=BodyFf[i];
    }
    return RIGHT;
}
int CJW_LWARobot::SetMainBodyFt(double BodyFt[3])
{
    for(int i=0;i<3;i++)
    {
        MainBody.Body.BodyF[i]=BodyFt[i];
    }
    return RIGHT;
}
/***********robot normal function**********************************************************************************/
//get the all angle state
int CJW_LWARobot::CheckALLLimit(void)
{
    int allerror=RIGHT;
    for(int i=0;i<BranchN;i++)
    {
        int legerror[BRANCH_N_MAX]={0};
        int theerror=BranchS[i].CheckJointLimit(legerror);
        if(theerror)
        {
            printf("ERROR: %d leg ：",i);
            int bodyN=BranchS[i].GetBodyN();
            for(int j=0;j<bodyN;j++)
            {
                if(legerror[j])
                {
                    printf(" joint%d is %s;",j,Joint_Error_Name[legerror[j]].c_str());
                }
            }
            allerror=allerror*2+1;
            printf("\n");
        }
        else
        {
            allerror=allerror*2;
        }
    }
    return allerror;
}
int CJW_LWARobot::GetALLAngleState(struct CJW_BasedJoint *out)
{
    for(int i=0;i<BranchN;i++)
    {
        BranchS[i].GetALLAngleState(&(out[i*BRANCH_BODY_MAX]));
    }
    return RIGHT;
}
int CJW_LWARobot::GetALLAngleP(double *out)
{
    for(int i=0;i<BranchN;i++)
    {
        BranchS[i].GetALLAngleP(&(out[i*BRANCH_BODY_MAX]));
    }
    return RIGHT;
}
int CJW_LWARobot::GetALLAngleV(double *out)
{
    for(int i=0;i<BranchN;i++)
    {
        BranchS[i].GetALLAngleV(&(out[i*BRANCH_BODY_MAX]));
    }
    return RIGHT;
}
int CJW_LWARobot::GetALLAngleA(double *out)
{
    for(int i=0;i<BranchN;i++)
    {
        BranchS[i].GetALLAngleA(&(out[i*BRANCH_BODY_MAX]));
    }
    return RIGHT;
}
int CJW_LWARobot::GetALLAngleF(double *out)
{
    for(int i=0;i<BranchN;i++)
    {
        BranchS[i].GetALLAngleF(&(out[i*BRANCH_BODY_MAX]));
    }
    return RIGHT;
}
//set the all all state
int CJW_LWARobot::SetALLAngleState(struct CJW_BasedJoint *out)
{
    for(int i=0;i<BranchN;i++)
    {
        BranchS[i].SetALLAngleState(&(out[i*BRANCH_BODY_MAX]));
    }
    return RIGHT;
}
int CJW_LWARobot::SetALLAngleP(double *out)
{
    for(int i=0;i<BranchN;i++)
    {
        BranchS[i].SetALLAngleP(&(out[i*BRANCH_BODY_MAX]));
    }
    return RIGHT;
}
int CJW_LWARobot::SetALLAngleV(double *out)
{
    for(int i=0;i<BranchN;i++)
    {
        BranchS[i].SetALLAngleV(&(out[i*BRANCH_BODY_MAX]));
    }
    return RIGHT;
}
int CJW_LWARobot::SetALLAngleA(double *out)
{
    for(int i=0;i<BranchN;i++)
    {
        BranchS[i].SetALLAngleA(&(out[i*BRANCH_BODY_MAX]));
    }
    return RIGHT;
}
int CJW_LWARobot::SetALLAngleF(double *out)
{
    for(int i=0;i<BranchN;i++)
    {
        BranchS[i].SetALLAngleF(&(out[i*BRANCH_BODY_MAX]));
    }
    return RIGHT;
}
/**based robot refresh************************************************************************************/
/**based robot refresh from angle to the G V and dynamic state*******************************************/
int CJW_LWARobot::RobotAngleToAllStateBody(void)
{
    double tempVS0[6]={0};
    memset(DynamicStateBody.dInertia,0,sizeof(DynamicStateBody.dInertia));
    memset(DynamicStateBody.Momentum,0,sizeof(DynamicStateBody.Momentum));
    MathFun.MyInertiaBasedGroup((double*)(MainBody.Body.M),(double*)(MainBody.Body.MG),DynamicStateBody.Inertia);
    for(int legi=0;legi<BranchN;legi++)
    {
        //based function
        BranchS[legi].ModelAngleToAllBodyStateRoot();
        //branch kinetic******************************************************
        BranchS[legi].ModelAnglePTransToTipG();
        BranchS[legi].ModelAngleVTransToTipV();
        BranchS[legi].ModelAngleFTransToTipF();
        BranchS[legi].GetModelTip(&TipRoot[legi]);
        TipBody[legi]=TipRoot[legi]; 
        MathFun.MyGCompositionG((double*)(BranchP[legi].G0),(double*)(TipRoot[legi].G),(double*)(TipBody[legi].G));
        //branch dynamic*******************************************************
        struct CJW_BranchDynamicState TheDynamicRoot=BranchS[legi].ModelGetDynamicStateRoot();
        //printf("the %d leg is \n",legi);Print_CJW_BranchDynamicState(&TheDynamicRoot);
        MathFun.MyInertiaDBasedGroup(TheDynamicRoot.Inertia,TheDynamicRoot.dInertia,(double*)(BranchP[legi].G0),tempVS0,DynamicStateBranch[legi].Inertia,DynamicStateBranch[legi].dInertia);
        //double AdgInvT[36];MathFun.MyEXPAdgInvT((double *)(BranchP[legi].G0),AdgInvT);
        //MathFun.MyMatrixMultiply(6,1,6,AdgInvT,TheDynamicRoot.Momentum,DynamicStateBranch[legi].Momentum);
        MathFun.MyEXPAdgInvTScrew((double *)(BranchP[legi].G0),TheDynamicRoot.Momentum,DynamicStateBranch[legi].Momentum);
        for(int ii=0;ii<36;ii++)
        {
            DynamicStateBody.Inertia[ii]=DynamicStateBody.Inertia[ii]+DynamicStateBranch[legi].Inertia[ii];
            DynamicStateBody.dInertia[ii]=DynamicStateBody.dInertia[ii]+DynamicStateBranch[legi].dInertia[ii];
        }
        for(int ii=0;ii<6;ii++)
        {
            DynamicStateBody.Momentum[ii]=DynamicStateBody.Momentum[ii]+DynamicStateBranch[legi].Momentum[ii];
        }
    }
    //printf("the robot is \n");Print_CJW_BranchDynamicState(&DynamicStateBody);
    //refresh G and V
    //MainAnglePTransToTipG();
    //MainAngleVTransToTipV();
    //MainAngleFTransToTipF();
    return RIGHT;
}
/**robot dynamic controller based on COI***************************/
int CJW_LWARobot::GetRobotDynamicStateBody(double COIInertia[36],double COIdInertia[36],double COIvelocity[6])
{
    double theV[6];
    MathFun.MySolveGS(6,DynamicStateBody.Inertia,DynamicStateBody.Momentum,theV);
    for(int ii=0;ii<36;ii++)
    {
        COIInertia[ii]=DynamicStateBody.Inertia[ii];
        COIdInertia[ii]=DynamicStateBody.dInertia[ii];
    }
    for(int ii=0;ii<6;ii++)
    {
        COIvelocity[ii]=theV[ii]+MainBody.Body.BodyV[ii];
    }
    return RIGHT;
}
/*get the body exp force based on accelation using the COI dynamic***************/
int CJW_LWARobot::GetRobotDynamicBodyForce(double AimA[6],double outF[6])
{
    double theV[6]={0};MathFun.MySolveGS(6,DynamicStateBody.Inertia,DynamicStateBody.Momentum,theV);
    double COIvelocity[6]={0};for(int ii=0;ii<6;ii++){COIvelocity[ii]=theV[ii]+MainBody.Body.BodyV[ii];}
    double AllForce[6]={0};MathFun.DynamicBaseCOI(DynamicStateBody.Inertia,DynamicStateBody.dInertia,COIvelocity,AimA,AllForce);
    double GravityAbsolute[6]={0,0,0,0,0,-WORLD_GRAVITY*DynamicStateBody.Inertia[35]};
    double GravityG[16]={MainBody.G0[0][0],MainBody.G0[1][0],MainBody.G0[2][0],-DynamicStateBody.Inertia[11]/DynamicStateBody.Inertia[35],
                         MainBody.G0[0][1],MainBody.G0[1][1],MainBody.G0[2][1],DynamicStateBody.Inertia[5]/DynamicStateBody.Inertia[35],
                         MainBody.G0[0][2],MainBody.G0[1][2],MainBody.G0[2][2],-DynamicStateBody.Inertia[4]/DynamicStateBody.Inertia[35],
                         0,0,0,1};
    //printf("the center mass is %.3f position is %.3f %.3f %.3f\n",DynamicStateBody.Inertia[35],GravityG[3],GravityG[7],GravityG[11]);
    //MathFun.Show(6,6,DynamicStateBody.Inertia);
    double GravityBody[6]={0};MathFun.MyEXPAdgInvTScrew(GravityG,GravityAbsolute,GravityBody);
    for(int i=0;i<6;i++){outF[i]=AllForce[i]-GravityBody[i];}
    return RIGHT;
}
int CJW_LWARobot::GetBranchiDynamicTipForce(int legi,double AimA[6],double TipF[6])//based on RobotAngleToAllStateBody();!!!!!
{
    double theV[6];MathFun.MySolveGS(6,DynamicStateBranch[legi].Inertia,DynamicStateBranch[legi].Momentum,theV);
    double COIvelocity[6]={0};for(int ii=0;ii<6;ii++){COIvelocity[ii]=theV[ii]+MainBody.Body.BodyV[ii];}
    double BranchFonBody[6]={0};MathFun.DynamicBaseCOI(DynamicStateBranch[legi].Inertia,DynamicStateBranch[legi].dInertia,COIvelocity,AimA,BranchFonBody);
    MathFun.MyEXPAdgTScrew((double*)(TipBody[legi].G),BranchFonBody,TipF);
    return RIGHT;
}
//set the trans between angle and tip*****************************************************************/
//get the absolute tip state
int CJW_LWARobot::GetMainModelTip(CJW_TipStruct* input)
{
    for(int i=0;i<BranchN;i++)
    {
        input[i]=TipAbsolute[i];
    }
    return RIGHT;
}
int CJW_LWARobot::GetMainModelTipOne(int legi,CJW_TipStruct* input)
{
    input[0]=TipAbsolute[legi];
    return RIGHT;
}
int CJW_LWARobot::GetMainModelTipGOne(int legi,double input[4][4])
{
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            input[i][j]=TipAbsolute[legi].G[i][j];
        }
    }
    return RIGHT;
}
int CJW_LWARobot::GetMainModelTipVOne(int legi,double input[6])
{
    for(int i=0;i<6;i++)
    {
        input[i]=TipAbsolute[legi].V[i];
    }
    return RIGHT;
}
int CJW_LWARobot::GetMainModelTipAOne(int legi,double input[6])
{
    for(int i=0;i<6;i++)
    {
        input[i]=TipAbsolute[legi].A[i];
    }
    return RIGHT;
}
int CJW_LWARobot::GetMainModelTipFOne(int legi,double input[6])
{
    for(int i=0;i<6;i++)
    {
        input[i]=TipAbsolute[legi].F[i];
    }
    return RIGHT;
}
//set the absolute tip state
int CJW_LWARobot::SetMainModelTip(CJW_TipStruct* input)
{
    for(int i=0;i<BranchN;i++)
    {
        TipAbsolute[i]=input[i];
    }
    return RIGHT;
}
int CJW_LWARobot::SetMainModelTipOne(int legi,CJW_TipStruct input)
{
    TipAbsolute[legi]=input;
    return RIGHT;
}
int CJW_LWARobot::SetMainModelTipGOne(int legi,double input[4][4])
{
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            TipAbsolute[legi].G[i][j]=input[i][j];
        }
    }
    return RIGHT;
}
int CJW_LWARobot::SetMainModelTipVOne(int legi,double input[6])
{
    for(int i=0;i<6;i++)
    {
        TipAbsolute[legi].V[i]=input[i];
    }
    return RIGHT;
}
int CJW_LWARobot::SetMainModelTipAOne(int legi,double input[6])
{
    for(int i=0;i<6;i++)
    {
        TipAbsolute[legi].A[i]=input[i];
    }
    return RIGHT;
}
int CJW_LWARobot::SetMainModelTipFOne(int legi,double input[6])
{
    for(int i=0;i<6;i++)
    {
        TipAbsolute[legi].F[i]=input[i];
    }
    return RIGHT;
}
//function of the robot
int CJW_LWARobot::MainAnglePTransToTipGOne(int legi)//based on RobotAngleToAllStateBody();!!!!!
{
    int error=RIGHT;
    MathFun.MyGCompositionG((double*)(MainBody.G0),(double*)(TipBody[legi].G),(double*)(TipAbsolute[legi].G));
    return error;
}
int CJW_LWARobot::MainAngleVTransToTipVOne(int legi)//based on RobotAngleToAllStateBody();!!!!!
{
    int error=RIGHT;
    double theV1[6]={0};double theA1[6]={0};
    MathFun.MyEXPAdgInvScrew((double*)(TipBody[legi].G),MainBody.Body.BodyV,theV1);
    MathFun.MyEXPAdgInvScrew((double*)(TipBody[legi].G),MainBody.Body.BodyA,theA1);
    for(int i=0;i<6;i++)
    {
        TipAbsolute[legi].V[i]=theV1[i]+TipRoot[legi].V[i];
        TipAbsolute[legi].A[i]=theV1[i]+TipRoot[legi].A[i];
    }
    return error;
}
int CJW_LWARobot::MainAngleFTransToTipFOne(int legi)
{
    int error=RIGHT;
    for(int i=0;i<6;i++){TipAbsolute[legi].F[i]=TipBody[legi].F[i];}
    return error;
}
int CJW_LWARobot::MainTipGTransToAnglePOne(int legi)
{
    int error=RIGHT;
    double theG1[4][4];MathFun.MyGCompositionG((double*)(MainBody.G0),(double*)(BranchP[legi].G0),(double*)theG1);
    double theBranchG[4][4];MathFun.MyGInvCompositionG((double*)theG1,(double*)(TipAbsolute[legi].G),(double*)theBranchG);
    BranchS[legi].SetModelTipG(theBranchG);
    error=BranchS[legi].ModelTipGTransToAngleP(MainBody.G0);
    //refresh
    /*BranchS[legi].GetModelTipG(theBranchG);
    MathFun.MyGCompositionG((double*)theG1,(double*)theBranchG,(double*)(TipAbsolute[legi].G));*/
    BranchS[legi].GetModelTipG(TipRoot[legi].G);
    MathFun.MyGCompositionG((double*)(BranchP[legi].G0),(double*)(TipRoot[legi].G),(double*)(TipBody[legi].G));
    MathFun.MyGCompositionG((double*)(MainBody.G0),(double*)(TipBody[legi].G),(double*)(TipAbsolute[legi].G));
    return error;
}
int CJW_LWARobot::MainTipVTransToAngleVOne(int legi)
{
    int error=RIGHT;
    //double theG1[4][4];BranchS[legi].GetModelTipG(theG1);
    //double theG2[4][4];MathFun.MyGCompositionG((double*)(BranchP[legi].G0),(double*)theG1,(double*)theG2);
    double theV1[6];MathFun.MyEXPAdgInvScrew((double*)(TipBody[legi].G),MainBody.Body.BodyV,theV1);
    double theA1[6];MathFun.MyEXPAdgInvScrew((double*)(TipBody[legi].G),MainBody.Body.BodyA,theA1);
    double theBranchV[6];double theBranchA[6];
    for(int i=0;i<6;i++)
    {
        theBranchV[i]=TipAbsolute[legi].V[i]-theV1[i];
        theBranchA[i]=TipAbsolute[legi].A[i]-theA1[i];
    }
    BranchS[legi].SetModelTipV(theBranchV);
    BranchS[legi].SetModelTipA(theBranchA);
    error=error+BranchS[legi].ModelTipVTransToAngleV();
    //refresh
    /*BranchS[legi].GetModelTipV(theBranchV);
    BranchS[legi].GetModelTipA(theBranchA);
    for(int i=0;i<6;i++)
    {
        TipAbsolute[legi].V[i]=theBranchV[i]+theV1[i];
        TipAbsolute[legi].A[i]=theBranchA[i]+theA1[i];
    }*/
    BranchS[legi].GetModelTipV(TipRoot[legi].V);
    BranchS[legi].GetModelTipA(TipRoot[legi].A);
    for(int i=0;i<6;i++)
    {
        TipBody[legi].V[i]=TipRoot[legi].V[i];
        TipBody[legi].A[i]=TipRoot[legi].A[i];
        TipAbsolute[legi].V[i]=TipBody[legi].V[i]+theV1[i];
        TipAbsolute[legi].A[i]=TipBody[legi].A[i]+theA1[i];
    }

    return error;
}
int CJW_LWARobot::MainTipFTransToAngleFOne(int legi)//based on RobotAngleToAllStateBody();!!!!!
{
    int error=RIGHT;
    BranchS[legi].SetModelTipF(TipAbsolute[legi].F);
    error=error+BranchS[legi].ModelTipFTransToAngleF();
    //refresh
    BranchS[legi].GetModelTipF(TipAbsolute[legi].F);
    return error;
}
int CJW_LWARobot::MainAnglePTransToTipG(void)
{
    int error=RIGHT;
    for(int legi=0;legi<BranchN;legi++){error=2*error+MainAnglePTransToTipGOne(legi);}
    return error;
}
int CJW_LWARobot::MainAngleVTransToTipV(void)//express the tip coordinate
{
    int error=RIGHT;
    for(int legi=0;legi<BranchN;legi++){error=2*error+MainAngleVTransToTipVOne(legi);}
    return error;
}
int CJW_LWARobot::MainAngleFTransToTipF(void)
{
    int error=RIGHT;
    for(int legi=0;legi<BranchN;legi++){error=2*error+MainAngleFTransToTipFOne(legi);}
    return error;
}
int CJW_LWARobot::MainTipGTransToAngleP(void)
{
    int error=RIGHT;
    for(int legi=0;legi<BranchN;legi++){error=2*error+MainTipGTransToAnglePOne(legi);}
    return error;
}
int CJW_LWARobot::MainTipVTransToAngleV(void)
{
    int error=RIGHT;
    for(int legi=0;legi<BranchN;legi++){error=2*error+MainTipVTransToAngleVOne(legi);}
    return error;
}
int CJW_LWARobot::MainTipFTransToAngleF(void)//express the tip coordinate
{
    int error=RIGHT;
    for(int legi=0;legi<BranchN;legi++){error=2*error+MainTipFTransToAngleFOne(legi);}
    return error;
}
/*****dynamic self function //based on RobotAngleToAllStateBody();!!!!!****************/
void CJW_LWARobot::MainResetPVKef(double dT,double* para)//机身位姿卡尔曼滤波初始化参数
{
    double bodyR[9] = { 1, 0, 0,
                        0, 1, 0,
                        0, 0, 1};
    double bodyp[3]={0,0,0};
    double bodyv[3]={0};//MathFun.MyRCompositionw(bodyR,&(MainBody.Body.BodyV[3]),bodyv);
    double footp[3*BRANCH_N_MAX];
    for(int branchi=0;branchi<BranchN;branchi++)
    {
        for(int ii=0;ii<3;ii++)
        {
            footp[branchi*3+ii]=TipBody[branchi].G[ii][3];
        }
    }
    PV_KF_Estimator.Initial(dT);
    PV_KF_Estimator.SetPara(para);
    PV_KF_Estimator.SetBodyLegState(bodyp,bodyv,footp);
}
int  CJW_LWARobot::MainTipPVTransToBodyPV(struct CJW_JointStruct* Ref,double dT)
{
    //信任参考数据的姿态和角速度
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++){MainBody.G0[i][j]=Ref->G0[i][j];}
        MainBody.Body.BodyA[i]=(MainBody.Body.BodyA[i]+(Ref->Body.BodyV[i]-MainBody.Body.BodyV[i])/dT)/2;
        MainBody.Body.BodyV[i]=Ref->Body.BodyV[i];
        MainBody.Body.BodyA[i+3]=Ref->Body.BodyA[i+3];
    }
    double bodyR[9] = {MainBody.G0[0][0], MainBody.G0[0][1], MainBody.G0[0][2],
                         MainBody.G0[1][0], MainBody.G0[1][1], MainBody.G0[1][2],
                         MainBody.G0[2][0], MainBody.G0[2][1], MainBody.G0[2][2]};
    double world_accelation[3];MathFun.MyRCompositionw(bodyR,&(MainBody.Body.BodyA[3]),world_accelation);
    double world_dP[3*BRANCH_N_MAX];double world_dv[3*BRANCH_N_MAX];
    double world_state[BRANCH_N_MAX]={0};
    int all_sup_flag=0;
    //求出分支相对机身状态
    for(int ID=0;ID<BranchN;ID++)
    {
        int ID3 = 3 * ID;
        world_state[ID] = BranchS[ID].GetState();
        if(world_state[ID]>=BRANCH_BOUNDING_STATE){all_sup_flag=1;}
        double Foot_R[9];double Foot_P[3];
        MathFun.MyGToRP((double *)(TipBody[ID].G), Foot_R, Foot_P);
        MathFun.MyRCompositionw(bodyR, Foot_P, &(world_dP[ID3]));
        double Foot_wr[3] = {0};MathFun.MyVector3Cross(MainBody.Body.BodyV, Foot_P, Foot_wr);
        double Foot_footv[3] = {0};MathFun.MyRCompositionw(Foot_R, &(TipBody[ID].V[3]), Foot_footv);
        double foottemp[3] = {Foot_wr[0] + Foot_footv[0], Foot_wr[1] + Foot_footv[1], Foot_wr[2] + Foot_footv[2]};
        MathFun.MyRCompositionw(bodyR,foottemp,&(world_dv[ID3]));
    }
    if(all_sup_flag==0){world_accelation[2]=-WORLD_GRAVITY;}//完全悬空重力加速度
    PV_KF_Estimator.SetIMUAcc(world_accelation);
    PV_KF_Estimator.SetLegObs(world_dP,world_dv,world_state);
    PV_KF_Estimator.EstimateOnce();
    double result_p[3]={MainBody.G0[0][3],MainBody.G0[1][3],MainBody.G0[2][3]};
    double result_v[3]={MainBody.Body.BodyV[3],MainBody.Body.BodyV[4],MainBody.Body.BodyV[5]};
    double result_fp[3*BRANCH_N_MAX];
    for(int branchi=0;branchi<BranchN;branchi++)
    {
        for(int ii=0;ii<3;ii++)
        {
            result_fp[branchi*3+ii]=TipAbsolute[branchi].G[ii][3];
        }
    }
    PV_KF_Estimator.GetBodyLegState(result_p,result_v,result_fp);
    MainBody.G0[0][3]=result_p[0];MainBody.G0[1][3]=result_p[1];MainBody.G0[2][3]=result_p[2];
    MathFun.MyRInvCompositionw(bodyR,result_v,&(MainBody.Body.BodyV[3]));
    MainAnglePTransToTipG();
    //get the KF the foot position
    for(int branchi=0;branchi<BranchN;branchi++)
    {
        for(int ii=0;ii<3;ii++)
        {
            TipAbsolute[branchi].G[ii][3]=result_fp[branchi*3+ii];
        }
    }
    MainAngleVTransToTipV();
    MainAngleFTransToTipF();
    //get the body F
    double RealBodyF0[6]={0};
    for(int branchi=0;branchi<BranchN;branchi++)
    {
        if(world_state[branchi]>=BRANCH_BOUNDING_STATE)
        {
            double temprealF[6]={0};
            MathFun.MyEXPAdgInvTScrew((double*)(TipBody[branchi].G),TipBody[branchi].F,temprealF);
            for(int ii=0;ii<6;ii++){RealBodyF0[ii]=RealBodyF0[ii]-temprealF[ii];}
        }
    }
    for(int ii=0;ii<6;ii++)
    {
        MainBody.Body.BodyF[ii]=RealBodyF0[ii];
    }

    // //估计位置和线速度
    // double TempP0[3]={0};
    // double TempV0[3]={0};
    // double SupportingFlag0=0;
    // double BodyR[9]={MainBody.G0[0][0],MainBody.G0[0][1],MainBody.G0[0][2],
    //                  MainBody.G0[1][0],MainBody.G0[1][1],MainBody.G0[1][2],
    //                  MainBody.G0[2][0],MainBody.G0[2][1],MainBody.G0[2][2]};
    // double thenowstate;
    // for(int ID=0;ID<BranchN;ID++)
    // {
    //     thenowstate=BranchS[ID].GetState();
    //     if(thenowstate>0)//supporting model
    //     {
    //         SupportingFlag0=SupportingFlag0+thenowstate;
    //         double Foot_R[9];double Foot_P[3];MathFun.MyGToRP((double*)(TipBody[ID].G),Foot_R,Foot_P);
    //         double Foot_dP[3];MathFun.MyRCompositionw(BodyR,Foot_P,Foot_dP);
    //         TempP0[0]=TempP0[0]+thenowstate*(TipAbsolute[ID].G[0][3]-Foot_dP[0]);
    //         TempP0[1]=TempP0[1]+thenowstate*(TipAbsolute[ID].G[1][3]-Foot_dP[1]);
    //         TempP0[2]=TempP0[2]+thenowstate*(TipAbsolute[ID].G[2][3]-Foot_dP[2]);
    //         double Foot_wr[3]={0};MathFun.MyVector3Cross(MainBody.Body.BodyV,Foot_P,Foot_wr);
    //         double Foot_footv[3]={0};MathFun.MyRCompositionw(Foot_R,&(TipBody[ID].V[3]),Foot_footv);
    //         TempV0[0]=TempV0[0]+thenowstate*(0-Foot_footv[0]-Foot_wr[0]);
    //         TempV0[1]=TempV0[1]+thenowstate*(0-Foot_footv[1]-Foot_wr[1]);
    //         TempV0[2]=TempV0[2]+thenowstate*(0-Foot_footv[2]-Foot_wr[2]);
    //     }
    // }
    // if(SupportingFlag0>0)
    // {
    //     for(int i=0;i<3;i++)
    //     {
    //         TempP0[i]=TempP0[i]/SupportingFlag0;
    //         TempV0[i]=TempV0[i]/SupportingFlag0;
    //         MainBody.G0[i][3]=TempP0[i];
    //         MainBody.Body.BodyV[i+3]=TempV0[i];
    //     }
    // }
    return RIGHT;
}
int CJW_LWARobot::GetExTipForceonQua(double ExpBodyF[6],double Qua_S[6],double Qua_W[3],double bodyF[6],double *outF)//based on RobotAngleToAllStateBody();!!!!!
{
    /*************************define the weight of quadprog*************************/
    ////the stand kesi: [Torque,Force];
    double my_S[36]={0};
    my_S[0]=Qua_S[0];my_S[7]=Qua_S[1];my_S[14]=Qua_S[2];
    my_S[21]=Qua_S[3];my_S[28]=Qua_S[4];my_S[35]=Qua_S[5];
    double my_W_based[3]={Qua_W[0],Qua_W[1],Qua_W[2]};
    /*******************************define the Quadprog structure************************/
    ////////////////////////////////////get the H and f
    int AllFN=0;int AllConN=0;
    double my_AT[BRANCH_N_MAX3][6]={0};double my_WW[BRANCH_N_MAX*3]={0};
    double my_b[6]={ExpBodyF[0],ExpBodyF[1],ExpBodyF[2],ExpBodyF[3],ExpBodyF[4],ExpBodyF[5]};
    double TheSpeciG[4][4]={{MainBody.G0[0][0],MainBody.G0[0][1],MainBody.G0[0][2],0},
                            {MainBody.G0[1][0],MainBody.G0[1][1],MainBody.G0[1][2],0},
                            {MainBody.G0[2][0],MainBody.G0[2][1],MainBody.G0[2][2],0},
                            {0,0,0,1}};
    for(int ID=0;ID<BranchN;ID++)
    {
        if(BranchS[ID].GetState()>=BRANCH_BOUNDING_STATE)//supporting model
        {
            //get the Ax=b A and b
            for(int ii=0;ii<3;ii++){TheSpeciG[ii][3]=MainBody.G0[ii][3]-TipAbsolute[ID].G[ii][3];}
            double TheA[6][6]={0};MathFun.MyEXPAdgT((double *)TheSpeciG,(double *)TheA);
            //get the A of the leg and size
            for(int ii=0;ii<3;ii++)
            {
                for(int jj=0;jj<6;jj++)
                {
                    my_AT[AllFN+ii][jj]=TheA[jj][ii+3];
                }
            }
            my_WW[AllFN]=my_W_based[0];my_WW[AllFN+1]=my_W_based[1];my_WW[AllFN+2]=my_W_based[2];
            AllFN=AllFN+3;AllConN=AllConN+4;
        }
    }
    if(AllFN==0)//no supporting leg
    {
        for(int ID=0;ID<BranchN*6;ID++){outF[ID]=0;}
        return RIGHT;
    }
    double my_A[BRANCH_N_MAX3*6]={0};MathFun.MyMatrixRorate(AllFN,6,(double*)my_AT,my_A);
    //get the H and f ///H=AT*S*A+W=====f=-aim_b*S*A///////////////////////////
    double my_SA[BRANCH_N_MAX3*6];MathFun.MyMatrixMultiply(6,AllFN,6,my_S,my_A,my_SA);
    double myH[BRANCH_N_MAX3*BRANCH_N_MAX3];MathFun.MyMatrixMultiply(AllFN,AllFN,6,(double*)my_AT,my_SA,myH);
    double myg[BRANCH_N_MAX3];MathFun.MyMatrixMultiply(1,AllFN,6,my_b,my_SA,myg);
    for(int i=0;i<AllFN;i++)
    {
        int j=i*(AllFN+1);
        myH[j]=myH[j]+my_WW[i];
        myg[i]=-myg[i];
    }
    //printf("data H is \n");MathFun.Show(AllFN,AllFN,(double*)myH);
    /////////////////////////////////get the lb ub A lbA ubA
    double mylb[BRANCH_N_MAX3]={0};double myub[12]={0};
    double myA[(BRANCH_N_MAX3)*(BRANCH_N_MAX*4)]={0};
    double mylbA[BRANCH_N_MAX*4]={0};double myubA[BRANCH_N_MAX*4]={0};double myubAMAX=2*FOOT_FXYZ_U*FOOT_FZ_MAX;
    for(int ID=0;ID<(AllFN/3);ID++)
    {
        int FindX=3*ID;int FindC=4*ID;
        int SID=FindC*AllFN+FindX;/*walking model*/
        mylb[FindX]=FOOT_FXY_MIN;   myub[FindX]=FOOT_FXY_MAX;
        mylb[FindX+1]=FOOT_FXY_MIN; myub[FindX+1]=FOOT_FXY_MAX;
        mylb[FindX+2]=FOOT_FZ_MIN;  myub[FindX+2]=FOOT_FZ_MAX;
        myA[SID]=1;          /*myA[SID+1]=0;*/         myA[SID+2]=FOOT_FXYZ_U;
        myA[SID+AllFN]=-1;   /*myA[SID+AllFN+1]=0;*/   myA[SID+AllFN+2]=FOOT_FXYZ_U;
        /*myA[SID+2*AllFN]=0;*/myA[SID+2*AllFN+1]=1;   myA[SID+2*AllFN+2]=FOOT_FXYZ_U;
        /*myA[SID+3*AllFN]=0;*/myA[SID+3*AllFN+1]=-1;  myA[SID+3*AllFN+2]=FOOT_FXYZ_U;
        /*mylbA[FindC]=0;*/    myubA[FindC]=myubAMAX;
        /*mylbA[FindC+1]=0;*/  myubA[FindC+1]=myubAMAX;
        /*mylbA[FindC+2]=0;*/  myubA[FindC+2]=myubAMAX;
        /*mylbA[FindC+3]=0;*/  myubA[FindC+3]=myubAMAX;
    }
    double myx[AllFN];
    int result=Quadprog_Solve(AllFN,AllConN,myH,myg,mylb,myub,myA,mylbA,myubA,myx);
    /*******************************return the foot force structure************************/
    if(result==0)//success
    {
        int FindF=0;
        for(int ID=0;ID<BranchN;ID++)
        {
            int LegID=ID*6;
            outF[LegID]=0;outF[LegID+1]=0;outF[LegID+2]=0;
            outF[LegID+3]=0;outF[LegID+4]=0;outF[LegID+5]=0;
            if(BranchS[ID].GetState()>=BRANCH_BOUNDING_STATE)//supproting gait
            {
                double theFootRT[9]={TipAbsolute[ID].G[0][0],TipAbsolute[ID].G[1][0],TipAbsolute[ID].G[2][0],
                                     TipAbsolute[ID].G[0][1],TipAbsolute[ID].G[1][1],TipAbsolute[ID].G[2][1],
                                     TipAbsolute[ID].G[0][2],TipAbsolute[ID].G[1][2],TipAbsolute[ID].G[2][2]};
                MathFun.MyMatrixMultiply(3,1,3,theFootRT,&(myx[FindF]),&(outF[LegID+3]));
                FindF=FindF+3;
            }
        }
        MathFun.MyMatrixMultiply(6,1,AllFN,my_A,myx,bodyF);
    }
    else
    {
        /**************************test the Ax-b**********************************************/
        MathFun.MyMatrixMultiply(6,1,AllFN,my_A,myx,bodyF);
        double TestDX[6];for(int i=0;i<6;i++){TestDX[i]=bodyF[i]-my_b[i];}
        printf("Ax-b is ");MathFun.Show(1,6,ExpBodyF);
        MathFun.Show(AllFN,1,myx);
    }
    return result;
}


/**************wheel joint design and control*********************************************************************/
int CJW_LWARobot::SetBranchWheelDirection(int branchi,double dir)
{
    //轮轴初始方向为Y正向，方向转轴初始为Z方向，几何解法
    //tan(th)=-(r00)*（-sin(a)）+r01*cos(a))/(r10*(-sin(a))+r11*cos(a));
    double the_dir=dir;
    if(dir>(M_PI/2)){the_dir=dir-M_PI;}
    else if(dir<(-M_PI/2)){the_dir=dir+M_PI;}
    if(fabs(the_dir)<(M_PI/2))
    {
        double BodySE2[4][4];MathFun.MySE3ToSE2((double *)MainBody.G0, (double *)BodySE2);
        double TipSPE[4][4];MathFun.MyGInvCompositionG((double *)BodySE2, (double *)TipAbsolute[branchi].G, (double *)TipSPE);
        double AA = TipSPE[0][0] + TipSPE[1][0] * tan(dir);
        double BB = TipSPE[0][1] + TipSPE[1][1] * tan(dir);
        the_dir = atan2(BB, AA);
    } 
    BranchS[branchi].SetOneAngleP(3,&the_dir);
    return RIGHT;
}
int CJW_LWARobot::SetAllWheelDirection(double *dir)
{
    for(int ii=0;ii<BranchN;ii++)
    {
        SetBranchWheelDirection(ii,dir[ii]);
    }
    return RIGHT;
}
double CJW_LWARobot::SetBranchWheelDirAuto(int branchi,double exp_v[6])
{
    //get the desired dir
    double TipBodyR[9];double TipBodyP[3];
    MathFun.MyGToRP((double*)TipBody[branchi].G,TipBodyR,TipBodyP);
    double foot_v[3]={0};MathFun.MyRInvCompositionw(TipBodyR,&TipBody[branchi].V[3],foot_v);
    double desired_wr[3];MathFun.MyVector3Cross(exp_v,TipBodyP,desired_wr);
    double desired_vall[3];
    for(int ii=0;ii<3;ii++){desired_vall[ii]=foot_v[ii]+exp_v[ii+3]+desired_wr[ii];}
    double outdir=atan2(desired_vall[1],desired_vall[0]);
    SetBranchWheelDirection(branchi,outdir);
    return outdir;
}
int CJW_LWARobot::SetAllWheelDirAuto(double exp_v[6])
{
    for(int ii=0;ii<BranchN;ii++)
    {
        SetBranchWheelDirAuto(ii,exp_v);
    }
    return RIGHT;    
}
int CJW_LWARobot::SetBranchWheelSpeed(int branchi,double speed)
{
    return BranchS[branchi].SetOneAngleV(4,&speed);
}
int CJW_LWARobot::SetAllWheelSpeed(double *Speed)
{
    for(int ii=0;ii<BranchN;ii++)
    {
        SetBranchWheelSpeed(ii,Speed[ii]);
    }
    return RIGHT;
}
int CJW_LWARobot::GetBranchWheelDirection(int branchi,double* dir)
{
    double the_dir=0;
    BranchS[branchi].GetOneAngleP(3,&the_dir);
    double d_screw[6]={0};
    BranchS[branchi].GetOneJStructScrew(3,d_screw);
    for(int ii=0;ii<6;ii++)
    {
        d_screw[ii]=d_screw[ii]*the_dir;
    }
    double d_R[9]={1,0,0,0,1,0,0,0,1};
    MathFun.MyExponent3ToR(d_screw,d_R);
    double w0_screw[6]={0};
    BranchS[branchi].GetOneJStructScrew(4,w0_screw);
    double wf_screw[3]={0};
    MathFun.MyRCompositionw(d_R,w0_screw,wf_screw);
    double ws_screw[3]={0};
    double BodySE2[4][4];MathFun.MySE3ToSE2((double*)MainBody.G0,(double*)BodySE2);
    double TipSPE[4][4];MathFun.MyGInvCompositionG((double*)BodySE2,(double*)TipAbsolute[branchi].G,(double*)TipSPE);
    for(int ii=0;ii<3;ii++)
    {
        ws_screw[ii]=TipSPE[ii][0]*wf_screw[0]+TipSPE[ii][1]*wf_screw[1]+TipSPE[ii][2]*wf_screw[2];
    }
    the_dir=-atan2(ws_screw[0],ws_screw[1]);
    if(the_dir>(M_PI/2.0))
    {
        the_dir=the_dir-M_PI;
    }
    else if(the_dir<(-M_PI/2.0))
    {
        the_dir=the_dir+M_PI;
    }
    dir[0]=the_dir;
    return RIGHT;
}
int CJW_LWARobot::GetAllWheelDirection(double *dir)
{
    for(int ii=0;ii<BranchN;ii++)
    {
        GetBranchWheelDirection(ii,&(dir[ii]));
    }
    return RIGHT;
}
int CJW_LWARobot::GetBranchWheelSpeed(int branchi,double* speed)
{
    return BranchS[branchi].GetOneAngleV(4,speed);
}
int CJW_LWARobot::GetAllWheelSpeed(double *Speed)
{
    for(int ii=0;ii<BranchN;ii++)
    {
        GetBranchWheelSpeed(ii,&(Speed[ii]));
    }
    return RIGHT;
}