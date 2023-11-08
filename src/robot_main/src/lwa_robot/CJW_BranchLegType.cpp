#include "CJW_BranchLegType.h"

void CJW_BranchLegType::CJW_Init(void)
{
    TheStructure->modelstate=BRANCH_LEG_MODEL;
    JN=3;
    TheStructure->JointN=JN;
    for(int i=0;i<(JN+1);i++)
    {
        for(int j=0;j<6;j++)
        {
            TheStructure->JStruct[i].Screw0[j]=0;
        }
    }
    TheStructure->JStruct[0].Screw0[0]=1;
    TheStructure->JStruct[1].Screw0[1]=1;
    TheStructure->JStruct[2].Screw0[1]=1;
    for(int i=0;i<JN;i++)
    {
        LengthL[i]=fabs(TheStructure->JStruct[i+1].G0[2][3]);
    }
    LY=TheStructure->JStruct[3].G0[1][3];
}
/************special on function**********************************************************/
//because the inversense only use the position after get angle will compute the orientation of tip
int  CJW_BranchLegType::TipGTransToAngleP(double BodyG[4][4])
{
    double thex=Tip.G[0][3];double they=Tip.G[1][3];double thez=Tip.G[2][3];
    double DisYZ0=sqrt(they*they+thez*thez-LY*LY);
    double DisYZ=DisYZ0-LengthL[0];
    double DisXYZ=sqrt(DisYZ*DisYZ+thex*thex);
    if(DisXYZ>(LengthL[1]+LengthL[2]))
    {
        return ERROR;
    }
    double out_temp[3];
    out_temp[0]=atan2(they,-thez)-atan2(LY,DisYZ0);
    out_temp[1]=-acos((1.0*LengthL[1]*LengthL[1]+DisXYZ*DisXYZ-LengthL[2]*LengthL[2])/(2.0*LengthL[1]*DisXYZ))-atan(1.0*thex/DisYZ);
    out_temp[2]=M_PI-acos((1.0*LengthL[1]*LengthL[1]+LengthL[2]*LengthL[2]-DisXYZ*DisXYZ)/(2.0*LengthL[1]*LengthL[2]));
    //printf("the out_temp is %.3f+%.3f+%.3f\n",out_temp[0],out_temp[1],out_temp[2]);
    if((TheStructure->Joint[0]->AngleLimit[0]<=out_temp[0])&&(out_temp[0]<=TheStructure->Joint[0]->AngleLimit[1]))
    {
        if((TheStructure->Joint[1]->AngleLimit[0]<=out_temp[1])&&(out_temp[1]<=TheStructure->Joint[1]->AngleLimit[1]))
        {

            if((TheStructure->Joint[2]->AngleLimit[0]<=out_temp[2])&&(out_temp[2]<=TheStructure->Joint[2]->AngleLimit[1]))
            {
                for(int i=0;i<JN;i++)
                {
                    TheStructure->Joint[i]->Angle=out_temp[i];
                }
                double th23=out_temp[1]+out_temp[2];
                Tip.G[0][0]=cos(th23);                  Tip.G[0][1]=0;                  Tip.G[0][2]=sin(th23);
                Tip.G[1][0]=sin(th23)*sin(out_temp[0]); Tip.G[1][1]=cos(out_temp[0]);   Tip.G[1][2]=-cos(th23)*sin(out_temp[0]);
                Tip.G[2][0]=-sin(th23)*cos(out_temp[0]);Tip.G[2][1]=sin(out_temp[0]);   Tip.G[2][2]=cos(th23)*cos(out_temp[0]);
                Tip.G[3][0]=0;Tip.G[3][1]=0;Tip.G[3][2]=0;Tip.G[3][3]=1;
                return RIGHT;
            }
        }
    }
    return ERROR;
}
//because the inversense only use the linear velocity after get angle will compute the rorate velocity of tip
int  CJW_BranchLegType::TipVTransToAngleV(void)
{
    double sinth3=sin(TheStructure->Joint[2]->Angle);
    if(fabs(sinth3)>EXP_ZERO)
    {
        double tempV[3]={0};
        double tempA[3]={0};
        double JacobianB[3][3]={0};
        double th23=TheStructure->Joint[1]->Angle+TheStructure->Joint[2]->Angle;
        JacobianB[0][0]=-LY*sin(th23);
        JacobianB[0][1]=-LengthL[2]-LengthL[1]*cos(TheStructure->Joint[2]->Angle);
        JacobianB[0][2]=-LengthL[2];
        JacobianB[1][0]=(LengthL[0]+LengthL[1]*cos(TheStructure->Joint[1]->Angle)+LengthL[2]*cos(th23));
        JacobianB[2][0]=LY*cos(th23);
        JacobianB[2][1]=-LengthL[1]*sin(TheStructure->Joint[2]->Angle);
        tempV[0]=Tip.V[4]/JacobianB[1][0];
        tempV[1]=(Tip.V[5]-JacobianB[2][0]*tempV[0])/JacobianB[2][1];
        tempV[2]=(Tip.V[3]-JacobianB[0][0]*tempV[0]-JacobianB[0][1]*tempV[1])/JacobianB[0][2];
        tempA[0]=Tip.A[4]/JacobianB[1][0];
        tempA[1]=(Tip.A[5]-JacobianB[2][0]*tempA[0])/JacobianB[2][1];
        tempA[2]=(Tip.A[3]-JacobianB[0][0]*tempA[0]-JacobianB[0][1]*tempA[1])/JacobianB[0][2];
        if((TheStructure->Joint[0]->VelocityLimit[0]<=tempV[0])&&(tempV[0]<=TheStructure->Joint[0]->VelocityLimit[1]))
        {
            if((TheStructure->Joint[1]->VelocityLimit[0]<=tempV[1])&&(tempV[1]<=TheStructure->Joint[1]->VelocityLimit[1]))
            {
                if((TheStructure->Joint[2]->VelocityLimit[0]<=tempV[2])&&(tempV[2]<=TheStructure->Joint[2]->VelocityLimit[1]))
                {
                    for(int i=0;i<JN;i++)
                    {
                        TheStructure->Joint[i]->Velocity=tempV[i];
                        TheStructure->Joint[i]->Accelation=tempA[i];
                    }
                    Tip.V[0]=cos(th23)*tempV[0];
                    Tip.V[1]=tempV[1]+tempV[2];
                    Tip.V[2]=sin(th23)*tempV[0];
                    Tip.A[0]=cos(th23)*tempA[0];
                    Tip.A[1]=tempA[1]+tempA[2];
                    Tip.A[2]=sin(th23)*tempA[0];
                    return RIGHT;
                }
            }
        }
    }
    return ERROR;
}
int  CJW_BranchLegType::AngleFTransToTipF(void)
{
    double sinth3=sin(TheStructure->Joint[2]->Angle);
    if(fabs(sinth3)>EXP_ZERO)
    {
        double tempV[3]={0};
        double JacobianB[3][3]={0};
        double th23=TheStructure->Joint[1]->Angle+TheStructure->Joint[2]->Angle;
        JacobianB[0][0]=-LY*sin(th23);
        JacobianB[0][1]=-LengthL[2]-LengthL[1]*cos(TheStructure->Joint[2]->Angle);
        JacobianB[0][2]=-LengthL[2];
        JacobianB[1][0]=(LengthL[0]+LengthL[1]*cos(TheStructure->Joint[1]->Angle)+LengthL[2]*cos(th23));
        JacobianB[2][0]=LY*cos(th23);
        JacobianB[2][1]=-LengthL[1]*sin(TheStructure->Joint[2]->Angle);
        Tip.F[3]=TheStructure->Joint[2]->Force/JacobianB[0][2];
        Tip.F[5]=(TheStructure->Joint[1]->Force-JacobianB[0][1]*Tip.F[3])/JacobianB[2][1];
        Tip.F[4]=(TheStructure->Joint[0]->Force-JacobianB[0][0]*Tip.F[3]-JacobianB[2][0]*Tip.F[5])/JacobianB[1][0];
        Tip.F[0]=0;
        Tip.F[1]=0;
        Tip.F[2]=0;
        return RIGHT;
    }
    return ERROR;
}

