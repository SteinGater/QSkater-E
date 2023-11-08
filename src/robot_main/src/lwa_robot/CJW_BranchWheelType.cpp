#include "CJW_BranchWheelType.h"

void CJW_BranchWheelType::CJW_Init(void)
{
    TheStructure->modelstate=BRANCH_WHEEL_MODEL;
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
    Equ_bodyR=Eigen::Matrix3d::Identity();
    Equ_footPN=CJW_Vector3::Zero();
    Equ_footyaw=0;

    /*if(LY<0)
    {
        struct timeval Test_start, Test_end, servo_start, servo_end;
        unsigned long Test_duration = 0;
        gettimeofday(&Test_start, NULL);
        double testG[4][4]={0};
        double testyaw=2.7;
        testG[0][0]=cos(testyaw);testG[0][1]=-sin(testyaw);
        testG[1][0]=sin(testyaw);testG[1][1]=cos(testyaw);
        testG[2][2]=1;testG[3][3]=1;
        double legyaw=0;
        Tip.G[0][0]=cos(legyaw); Tip.G[0][1]=-sin(legyaw); Tip.G[0][2]=0; Tip.G[0][3]=0;
        Tip.G[1][0]=sin(legyaw); Tip.G[1][1]=cos(legyaw); Tip.G[1][2]=0; Tip.G[1][3]=0;
        Tip.G[2][0]=0; Tip.G[2][1]=0; Tip.G[2][2]=1; Tip.G[2][3]=-0.3;
        Tip.G[3][0]=0; Tip.G[3][1]=0; Tip.G[3][2]=0; Tip.G[3][3]=1;
        int TestN=1000;
        for(int testi=0;testi<TestN;testi++)
        {
            //std::cout<<"theN is %d"<<testi<<std::endl;
            legyaw=M_PI/4*sin(1.0*testi/TestN*M_PI);
            Tip.G[0][0]=cos(legyaw); Tip.G[0][1]=-sin(legyaw); Tip.G[0][2]=0;
            Tip.G[1][0]=sin(legyaw); Tip.G[1][1]=cos(legyaw); Tip.G[1][2]=0;
            Tip.G[1][3]=1.0*TestN*0.1/TestN-0.1;
            Inverted_Numerical(testG);
        }
        gettimeofday(&Test_end, NULL);
        Test_duration = (CLOCKS_PER_SEC * (Test_end.tv_sec - Test_start.tv_sec) + (Test_end.tv_usec - Test_start.tv_usec));//
        printf("using out the time is %d\n", Test_duration);
    }*/
}
/************special on function**********************************************************/
int  CJW_BranchWheelType::AngleFTransToTipF(void)
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
//because the inversense only use the position after get angle will compute the orientation of tip
int  CJW_BranchWheelType::TipGTransToAngleP(double BodyG[4][4])
{
    if(INVERTED_KINE_MERHOD)
    {
        return Inverted_Numerical(BodyG);
    }
    else
    {
        return Inverted_Simple(BodyG);
    }
}

int  CJW_BranchWheelType::Inverted_Simple(double BodyG[4][4])
{
    double thex=Tip.G[0][3];double they=Tip.G[1][3];double thez=Tip.G[2][3];
    double DisYZ0=sqrt(they*they+thez*thez-LY*LY);
    double DisYZ=DisYZ0-LengthL[0];
    double DisXYZ=sqrt(DisYZ*DisYZ+thex*thex);
    if(DisXYZ>(LengthL[1]+LengthL[2]))
    {
        return ERROR;
    }
    double out_temp[4];
    out_temp[0]=atan2(they,-thez)-atan2(LY,DisYZ0);
    out_temp[1]=+acos((1.0*LengthL[1]*LengthL[1]+DisXYZ*DisXYZ-LengthL[2]*LengthL[2])/(2.0*LengthL[1]*DisXYZ))-atan(1.0*thex/DisYZ);
    out_temp[2]=-M_PI+acos((1.0*LengthL[1]*LengthL[1]+LengthL[2]*LengthL[2]-DisXYZ*DisXYZ)/(2.0*LengthL[1]*LengthL[2]));
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
int  CJW_BranchWheelType::Inverted_Numerical(double BodyG[4][4])
{
    //record the initial data and solver parameters;
    Eigen::Matrix3d TipR;
    CJW_Vector3 TipP;
    for (int ii = 0; ii < 3; ii++)
    {
        TipP(ii,0)=Tip.G[ii][3];
        for (int jj = 0; jj < 3; jj++)
        {
            Equ_bodyR(ii,jj) = BodyG[ii][jj];
            TipR(ii,jj)=Tip.G[ii][jj];
        }
    }
    Equ_footPN=Equ_bodyR*TipP-(Equ_rw+Equ_bw)*Equ_bodyR.block<1,3>(2,0).transpose();
    Eigen::Matrix3d footSd=Equ_bodyR*TipR;
    Equ_footyaw=-atan2(footSd(0,1),footSd(1,1));//std::cout<<"the yaw::="<<Equ_footyaw<<std::endl;
    //simple invertd kinematics for the initial values of numerical solver;
    double thex=Tip.G[0][3];
    double they=Tip.G[1][3];
    double thez=Tip.G[2][3];
    double DisYZ0=sqrt(they*they+thez*thez-LY*LY);
    double DisYZ=DisYZ0-LengthL[0];
    double DisXYZ=sqrt(DisYZ*DisYZ+thex*thex);
    if(DisXYZ>(LengthL[1]+LengthL[2]))
    {
        return ERROR;
    }
    Eigen::Matrix<double,Eigen::Dynamic,1> xx0;
    xx0.resize(4,1);
    xx0(0)=atan2(they,-thez)-atan2(LY,DisYZ0);
    xx0(1)=+acos((1.0*LengthL[1]*LengthL[1]+DisXYZ*DisXYZ-LengthL[2]*LengthL[2])/(2.0*LengthL[1]*DisXYZ))-atan(1.0*thex/DisYZ);
    xx0(2)=-M_PI+acos((1.0*LengthL[1]*LengthL[1]+LengthL[2]*LengthL[2]-DisXYZ*DisXYZ)/(2.0*LengthL[1]*LengthL[2]));
    xx0(3)=-atan2(TipR(0,1),TipR(1,1))*1;
    if(xx0(3)>M_PI/2){xx0(3)=xx0(3)-M_PI;}
    else if(xx0(3)< -M_PI/2){xx0(3)=xx0(3)+M_PI;}

    //solver the nonlinear equation
    if(fabs(xx0(1))>0.1)//the singular is not suitable the numerical solver
    {
        //std::cout<<"initial x=::"<<xx0.transpose()<<std::endl;
        Solver(xx0);
        //std::cout<<"solver x=::"<<xx0.transpose()<<"=iter::"<<solver.iter<<std::endl;
    }
    if((TheStructure->Joint[0]->AngleLimit[0]<=xx0(0))&&(xx0(0)<=TheStructure->Joint[0]->AngleLimit[1]))
    {
        if((TheStructure->Joint[1]->AngleLimit[0]<=xx0(1))&&(xx0(1)<=TheStructure->Joint[1]->AngleLimit[1]))
        {
            if((TheStructure->Joint[2]->AngleLimit[0]<=xx0(2))&&(xx0(2)<=TheStructure->Joint[2]->AngleLimit[1]))
            {
                for(int i=0;i<JN;i++)
                {
                    TheStructure->Joint[i]->Angle=xx0(i);
                }
                double th23=xx0(1)+xx0(2);
                Tip.G[0][0]=cos(th23);                  Tip.G[0][1]=0;                  Tip.G[0][2]=sin(th23);
                Tip.G[1][0]=sin(th23)*sin(xx0(0)); Tip.G[1][1]=cos(xx0(0));   Tip.G[1][2]=-cos(th23)*sin(xx0(0));
                Tip.G[2][0]=-sin(th23)*cos(xx0(0));Tip.G[2][1]=sin(xx0(0));   Tip.G[2][2]=cos(th23)*cos(xx0(0));
                Tip.G[3][0]=0;Tip.G[3][1]=0;Tip.G[3][2]=0;Tip.G[3][3]=1;
                return RIGHT;
            }
        }
    }
    return ERROR;
}
//because the inversense only use the linear velocity after get angle will compute the rorate velocity of tip
int  CJW_BranchWheelType::TipVTransToAngleV(void)
{
    double sinth3=sin(TheStructure->Joint[2]->Angle);
    if(fabs(sinth3)>EXP_ZERO)
    {
        double tempV[4]={0};
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

/***********optimization function**********************************************************/
void CJW_BranchWheelType::cost_function(const InputType &x, ValueType &y)
{
    //one leg forward kinematic without body information
    Eigen::Matrix3d footR_root;
    CJW_Vector3 footP_root;
    double x12=x(1)+x(2);
    footR_root(0,0)=cos(x12)*cos(x(3));
    footR_root(0,1)=-cos(x12)*sin(x(3));
    footR_root(0,2)=sin(x12);
    footR_root(1,0)=cos(x(0))*sin(x(3))+cos(x(3))*sin(x(0))*sin(x12);
    footR_root(1,1)=cos(x(0))*cos(x(3))-sin(x(3))*sin(x(0))*sin(x12);
    footR_root(1,2)=-cos(x12)*sin(x(0));
    footR_root(2,0)=sin(x(0))*sin(x(3))-cos(x(4))*cos(x(0))*sin(x12);
    footR_root(2,1)=sin(x(0))*cos(x(3))+sin(x(3))*cos(x(0))*sin(x12);
    footR_root(2,2)=cos(x12)*cos(x(0));
    footP_root(0,0)=-LengthL[2]*sin(x12)-LengthL[1]*sin(x(1));
    double theLL=LengthL[0]+LengthL[1]*cos(x(1))+LengthL[2]*cos(x12);
    footP_root(1,0)=LY*cos(x(0))+sin(x(0))*theLL;
    footP_root(2,0)=LY*sin(x(0))-cos(x(0))*theLL;
    CJW_Vector3 footPW=Equ_bodyR*footP_root;
    Eigen::Matrix3d footR_S=Equ_bodyR*footR_root;
    //design the cost function
    CJW_Vector3 vec_z;vec_z<<0,0,1;
    y(0,0)=footR_S(0,1)*cos(Equ_footyaw)+footR_S(1,1)*sin(Equ_footyaw);
    y.block<3,1>(1,0)=Equ_rw*(footR_S.block<3,1>(0,1)*footR_S(2,1)-vec_z)
    +(footPW-Equ_footPN-vec_z*Equ_bw)*sqrt(1-footR_S(2,1)*footR_S(2,1));
}
