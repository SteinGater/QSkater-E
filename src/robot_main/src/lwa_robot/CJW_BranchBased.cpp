#include "CJW_BranchBased.h"



/************based on function**********************************************************/
void CJW_BranchBased::CJW_Init(void)
{

}

//show the branch struct
void CJW_BranchBased::ShowStruct(void)
{
    printf("The %d model with %d DoF is description as:\n",TheStructure->modelstate,TheStructure->JointN);
    printf("Base");
    for(int i=0;i<TheStructure->JointN;i++)
    {
        printf("--%s--%s",TheStructure->Joint[i]->name.c_str(),TheStructure->JStruct[i].name.c_str());
    }
    printf("--%s\n",TheStructure->JStruct[TheStructure->JointN].name.c_str());
}
//get state of this branch model
int  CJW_BranchBased::GetModel(void)
{
    return TheStructure->modelstate;
}
//int  SetModel(int input)
//{
//TheStructure->modelstate=input;
//}
double  CJW_BranchBased::GetState(void)
{
    return TheStructure->State;
}
int  CJW_BranchBased::SetState(double input)
{
    TheStructure->State=input;
    return RIGHT;
}
/*********** the interface for the branch model*********************************************************************/
void CJW_BranchBased::ShowBranch(void)
{
    Print_CJW_BranchModel(TheStructure);
}
//get the angle state
int CJW_BranchBased::GetAngleState(struct CJW_BasedJoint *out)
{
    for(int i=0;i<TheStructure->JointN;i++)
    {
        out[i]=*(TheStructure->Joint[i]);
    }
    return RIGHT;
}
int CJW_BranchBased::GetAngleP(double *out)
{
    for(int i=0;i<TheStructure->JointN;i++)
    {
        out[i]=TheStructure->Joint[i]->Angle;
    }
    return RIGHT;
}
int CJW_BranchBased::GetAngleV(double *out)
{
    for(int i=0;i<TheStructure->JointN;i++)
    {
        out[i]=TheStructure->Joint[i]->Velocity;
    }
    return RIGHT;
}
int CJW_BranchBased::GetAngleA(double *out)
{
    for(int i=0;i<TheStructure->JointN;i++)
    {
        out[i]=TheStructure->Joint[i]->Accelation;
    }
    return RIGHT;
}
int CJW_BranchBased::GetAngleF(double *out)
{
    for(int i=0;i<TheStructure->JointN;i++)
    {
        out[i]=TheStructure->Joint[i]->Force;
    }
    return RIGHT;
}
//set the banch state
int CJW_BranchBased::SetAngleState(struct CJW_BasedJoint *out)
{
    double thedata[BRANCH_BODY_MAX]={0};
    for(int i=0;i<TheStructure->JointN;i++){thedata[i]=out[i].Angle;}
    SetAngleP(thedata);
    for(int i=0;i<TheStructure->JointN;i++){thedata[i]=out[i].Velocity;}
    SetAngleV(thedata);
    for(int i=0;i<TheStructure->JointN;i++){thedata[i]=out[i].Accelation;}
    SetAngleA(thedata);
    for(int i=0;i<TheStructure->JointN;i++){thedata[i]=out[i].Force;}
    SetAngleF(thedata);
    return RIGHT;
}
int CJW_BranchBased::SetAngleP(double *out)
{
    for(int i=0;i<TheStructure->JointN;i++)
    {
        if(TheStructure->Joint[i]->AngleLimit[0]>out[i])
        {
            TheStructure->Joint[i]->Angle=TheStructure->Joint[i]->AngleLimit[0];
        }
        else if(TheStructure->Joint[i]->AngleLimit[1]<out[i])
        {
            TheStructure->Joint[i]->Angle=TheStructure->Joint[i]->AngleLimit[1];
        }
        else
        {
            TheStructure->Joint[i]->Angle=out[i];
        }
    }
    return RIGHT;
}
int CJW_BranchBased::SetAngleV(double *out)
{
    for(int i=0;i<TheStructure->JointN;i++)
    {
        if(TheStructure->Joint[i]->VelocityLimit[0]>out[i])
        {
            TheStructure->Joint[i]->Velocity=TheStructure->Joint[i]->VelocityLimit[0];
        }
        else if(TheStructure->Joint[i]->VelocityLimit[1]<out[i])
        {
            TheStructure->Joint[i]->Velocity=TheStructure->Joint[i]->VelocityLimit[1];
        }
        else
        {
            TheStructure->Joint[i]->Velocity=out[i];
        }
    }
    return RIGHT;
}
int CJW_BranchBased::SetAngleA(double *out)
{
    for(int i=0;i<TheStructure->JointN;i++)
    {
        if(TheStructure->Joint[i]->AccelationLimit[0]>out[i])
        {
            TheStructure->Joint[i]->Accelation=TheStructure->Joint[i]->AccelationLimit[0];
        }
        else if(TheStructure->Joint[i]->AccelationLimit[1]<out[i])
        {
            TheStructure->Joint[i]->Accelation=TheStructure->Joint[i]->AccelationLimit[1];
        }
        else
        {
            TheStructure->Joint[i]->Accelation=out[i];
        }
    }
    return RIGHT;
}
int CJW_BranchBased::SetAngleF(double *out)
{
    for(int i=0;i<TheStructure->JointN;i++)
    {
        if(TheStructure->Joint[i]->ForceLimit[0]>out[i])
        {
            TheStructure->Joint[i]->Force=TheStructure->Joint[i]->ForceLimit[0];
        }
        else if(TheStructure->Joint[i]->ForceLimit[1]<out[i])
        {
            TheStructure->Joint[i]->Force=TheStructure->Joint[i]->ForceLimit[1];
        }
        else
        {
            TheStructure->Joint[i]->Force=out[i];
        }
    }
    return RIGHT;
}
//get the tip state
int CJW_BranchBased::GetTip(struct CJW_TipStruct* Tip0)
{
    Tip0[0]=Tip;
    return RIGHT;
}
int CJW_BranchBased::GetTipG(double TipG[4][4])
{
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            TipG[i][j]=Tip.G[i][j];
        }
    }
    return RIGHT;
}
int CJW_BranchBased::GetTipP(double TipP[3])
{
    for(int i=0;i<3;i++)
    {
        TipP[i]=Tip.G[i][3];
    }
    return RIGHT;
}
int CJW_BranchBased::GetTipV(double TipV[6])
{
    for(int i=0;i<6;i++)
    {
        TipV[i]=Tip.V[i];
    }
    return RIGHT;
}
int CJW_BranchBased::GetTipA(double TipA[6])
{
    for(int i=0;i<6;i++)
    {
        TipA[i]=Tip.A[i];
    }
    return RIGHT;
}
int CJW_BranchBased::GetTipF(double TipF[6])
{
    for(int i=0;i<4;i++)
    {
        TipF[i]=Tip.F[i];
    }
    return RIGHT;
}
//set the tip state
int CJW_BranchBased::SetTip(struct CJW_TipStruct* Tip0)
{
    Tip=Tip0[0];
    return RIGHT;
}
int CJW_BranchBased::SetTipG(double TipG[4][4])
{
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            Tip.G[i][j]=TipG[i][j];
        }
    }
    return RIGHT;
}
int CJW_BranchBased::SetTipP(double TipP[3])
{
    for(int i=0;i<3;i++)
    {
        Tip.G[i][3]=TipP[i];
    }
    return RIGHT;
}
int CJW_BranchBased::SetTipV(double TipV[6])
{
    for(int i=0;i<6;i++)
    {
        Tip.V[i]=TipV[i];
    }
    return RIGHT;
}
int CJW_BranchBased::SetTipA(double TipA[6])
{
    for(int i=0;i<6;i++)
    {
        Tip.A[i]=TipA[i];
    }
    return RIGHT;
}
int CJW_BranchBased::SetTipF(double TipF[6])
{
    for(int i=0;i<6;i++)
    {
        Tip.F[i]=TipF[i];
    }
    return RIGHT;
}
/***********compute the state of the model branch for the robot*********************************************************************/
//motion and dynamic function to improve the speed for all forward compute function
int CJW_BranchBased::AngleToAllBodyStateRoot(void)
{
    //compute kinetic parameter******************************
    //get every body G and V on the root coordinate
    for(int i=0;i<(TheStructure->JointN);i++)
    {
        //get the joint Group
        double theEXP[6];for(int thei=0;thei<6;thei++){theEXP[thei]=TheStructure->Joint[i]->Angle*TheStructure->JStruct[i].Screw0[thei];}
        double theG[4][4];MathFun.MyExponent4ToG(theEXP,(double*)theG);
        if(i>0)
        {
            double theG1[4][4];MathFun.MyGCompositionG((double*)(TheStructure->JStruct[i].G0),(double*)theG,(double*)theG1);
            MathFun.MyGCompositionG((double*)(BodyStateRoot[i-1].G),(double*)theG1,(double*)(BodyStateRoot[i].G));
        }
        else
        {
            MathFun.MyGCompositionG((double*)(TheStructure->JStruct[i].G0),(double*)theG,(double*)(BodyStateRoot[i].G));
        }
        //get Jocabion of this Joint root
        MathFun.MyEXPAdgScrew((double*)(BodyStateRoot[i].G),TheStructure->JStruct[i].Screw0,BodyStateRoot[i].V);
    }
    //compute the root Jocabin
    for(int i=0;i<6;i++){for(int j=0;j<(TheStructure->JointN);j++){JocabinRoot[i*TheStructure->JointN+j]=BodyStateRoot[j].V[i];}}
    //compute dynamic parameter******************************
    memset(DynamicStateRoot.Inertia,0,sizeof(DynamicStateRoot.Inertia));
    memset(DynamicStateRoot.dInertia,0,sizeof(DynamicStateRoot.dInertia));
    memset(DynamicStateRoot.Momentum,0,sizeof(DynamicStateRoot.Momentum));
    double tempV[6]={0};double thedM0[36]={0};
    for(int i=0;i<(TheStructure->JointN);i++)
    {
        for(int thej=0;thej<6;thej++){tempV[thej]=tempV[thej]+TheStructure->Joint[i]->Velocity*BodyStateRoot[i].V[thej];}
        struct CJW_BranchDynamicState ThisDynamic;
        MathFun.MyInertiaDBasedGroup((double *)(TheStructure->JStruct[i].Body.M),thedM0,(double *)(BodyStateRoot[i].G),tempV,ThisDynamic.Inertia,ThisDynamic.dInertia);//优化代码
        MathFun.MyMatrixMultiply(6,1,6,ThisDynamic.Inertia,tempV,ThisDynamic.Momentum);
        //Print_CJW_BranchDynamicState(&ThisDynamic);
        for(int Mi=0;Mi<36;Mi++)
        {
            DynamicStateRoot.Inertia[Mi]=DynamicStateRoot.Inertia[Mi]+ThisDynamic.Inertia[Mi];
            DynamicStateRoot.dInertia[Mi]=DynamicStateRoot.dInertia[Mi]+ThisDynamic.dInertia[Mi];
            if(Mi<6)
            {
                DynamicStateRoot.Momentum[Mi]=DynamicStateRoot.Momentum[Mi]+ThisDynamic.Momentum[Mi];
            }
        }
    }
    return RIGHT;
}
//express function after running the function AngleToAllBodyStateRoot();
struct CJW_BranchDynamicState  CJW_BranchBased::GetDynamicStateRoot(void)
{
    return DynamicStateRoot;
}
int  CJW_BranchBased::GetJacobianTip(double *outJ)
{
    double TipAdgInv[36];MathFun.MyEXPAdgInv((double *)Tip.G,TipAdgInv);
    MathFun.MyMatrixMultiply(6,TheStructure->JointN,6,TipAdgInv,JocabinRoot,outJ);
    return TheStructure->JointN;
}
int  CJW_BranchBased::GetGravityComForJoint(double* bodyR,double *JointF)
{
    double thisMassG[4][4];
    for(int i=0;i<(TheStructure->JointN);i++)
    {
        JointF[i]=0;
        MathFun.MyGCompositionG((double*)(BodyStateRoot[i].G),(double*)(TheStructure->JStruct[i].Body.MG),(double*)thisMassG);
        double thisMassP[3]={thisMassG[0][3],thisMassG[1][3],thisMassG[2][3]};
        double thisscrew[6]={0,0,0,
                           TheStructure->JStruct[i].Body.M[5][5]*bodyR[6],
                           TheStructure->JStruct[i].Body.M[5][5]*bodyR[7],
                           TheStructure->JStruct[i].Body.M[5][5]*bodyR[8]};
        MathFun.MyVector3Cross(thisMassP,&(thisscrew[3]),thisscrew);
        for(int j=0;j<=i;j++)
        {
            double result=0;MathFun.MyMatrixMultiply(1,1,6,thisscrew,BodyStateRoot[j].V,&result);
            JointF[j]=JointF[j]+result*WORLD_GRAVITY;
        }
    }
    return TheStructure->JointN;
}
int CJW_BranchBased::AnglePTransToTipG(void)
{//trans between the angle and the tip
    MathFun.MyGCompositionG((double*)(BodyStateRoot[TheStructure->JointN-1].G),(double*)(TheStructure->JStruct[TheStructure->JointN].G0),(double*)(Tip.G));
    return RIGHT;
}
int  CJW_BranchBased::AngleVTransToTipV(void)
{//the velocity of tip is expressed in the tip coordinate
    double RootV[6]={0};
    double RootA[6]={0};
    for(int i=0;i<6;i++)
    {
        for(int j=0;j<(TheStructure->JointN);j++)
        {
            RootV[i]=RootV[i]+TheStructure->Joint[j]->Velocity*BodyStateRoot[j].V[i];
            RootA[i]=RootA[i]+TheStructure->Joint[j]->Accelation*BodyStateRoot[j].V[i];
        }
    }
    MathFun.MyEXPAdgInvScrew((double *)Tip.G,RootV,Tip.V);
    MathFun.MyEXPAdgInvScrew((double *)Tip.G,RootA,Tip.A);
    return RIGHT;
}
int  CJW_BranchBased::TipFTransToAngleF(void)//based on AngleToAllBodyStateRoot();!!!!!
{
    //double TipAdgInvT[36];MathFun.MyEXPAdgInvT((double *)Tip.G,TipAdgInvT);
    double F_S[6];//MathFun.MyMatrixMultiply(6,1,6,TipAdgInvT,Tip.F,F_S);
    MathFun.MyEXPAdgInvTScrew((double *)Tip.G,Tip.F,F_S);
    for(int i=0;i<(TheStructure->JointN);i++)
    {
        TheStructure->Joint[i]->Force=0;
        for(int j=0;j<6;j++)
        {
            TheStructure->Joint[i]->Force=TheStructure->Joint[i]->Force+BodyStateRoot[i].V[j]*F_S[j];
        }
    }
    return RIGHT;
}
