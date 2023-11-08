#include "CJW_GaitToRobotBased.h"

/***********************************************************************/
int CJW_GaitToRobotBased::Init(CJW_LWARobot* exprobot,CJW_LWARobot* realrobot,CJW_GaitBased* gait)
{
    ExpRobot=exprobot;
    RealRobot=realrobot;
    ThisGait=gait;
    int TheN=RealRobot->GetBranchN();
    if(TheN!=ThisGait->GetGaitN()) {return ERROR;}
    RealRobot->GetMainBodyState(&GaitInitBody0);
    RealRobot->GetMainModelTip(GaitInitTip0);
    RealRobot->GetALLAngleState(GaitInitJoint0);
    RealRobot->GetMainBodyState(&GaitInitBody1);
    RealRobot->GetMainModelTip(GaitInitTip1);
    RealRobot->GetALLAngleState(GaitInitJoint1);
    RealRobot->GetMainBodyState(&GaitInitBody2);
    RealRobot->GetMainModelTip(GaitInitTip2);
    RealRobot->GetALLAngleState(GaitInitJoint2);
    RealRobot->GetMainBodyState(&ComBody);
    RealRobot->GetMainModelTip(ComTip);
    RealRobot->GetMainBodyState(&Gait_Body);
    RealRobot->GetMainModelTip(Gait_Tip);
    RealRobot->GetMainBodyState(&Gait_BodySE2);
    RealRobot->GetMainModelTip(Gait_TipSE2);
    Gait_T0=4;
    Gait_StepH=0;
    GaitNum=0;
    for(int legi=0;legi<BRANCH_N_MAX;legi++)
    {
        ComBraState[legi]=BRANCH_SUP_STATE;
        ComBraType[legi]=TIP_DATA_SPE;
        GaitSwingflag[legi]=BRANCH_SUP_STATE;
        ExpWheelDir[legi]=0;
        ExpWheelSpeed[legi]=0;
    }
    GaitType=0;
    MoveMode=DESIGN_MOVE_DECOUPE;
    BodyType=DESIGN_CUBIC;
    SwingType=DESIGN_CYCLOID;
    for(int ii=0;ii<6;ii++)
    {
        Gait_Margin[ii]=0;
        ControlBodyFKp[ii]=0;
        ControlBodyFKd[ii]=0;
        ControlBodyAccMax[ii]=0;
        ControlBodyAccMin[ii]=0;
        ControlQua_S[ii]=0;
        ControlBodyVKp[ii]=0;
    }
    for(int ii=0;ii<3;ii++)
    {
        Gait_Dir[ii]=0;
        ControlQua_W[ii]=0;
        ControlInsideKp[ii]=0;
        ControlInsideKd[ii]=0;
        ControlSwingKp[ii]=0;
        ControlSwingKd[ii]=0;
    }
    ExpWheelStructDir=M_PI/4;
    return RIGHT;
}
/*******************************contact the robot and gait***********************************/
void CJW_GaitToRobotBased::SetExpRobot(CJW_LWARobot* robot)
{
    ExpRobot=robot;
}
void CJW_GaitToRobotBased::SetRealRobot(CJW_LWARobot* robot)
{
    RealRobot=robot;
}
void CJW_GaitToRobotBased::SetGait(CJW_GaitBased* gait)
{
    ThisGait=gait;
}

/*************************************************************************************************************/
/*************************************************************************************************************/
/*******************************controller of the robot**********************************************************************/
int CJW_GaitToRobotBased::SetControlParameter(double BodyFKp[6],double BodyFKd[6],double BodyVKp[6],double Qua_S[6],double Qua_W[3],double BodyAccMax[6],double BodyAccMin[6],
                                  double InsideKp[3],double InsideKd[3],double SwingKp[3],double SwingKd[3])
{
    for(int n=0;n<6;n++)
    {
        ControlBodyFKp[n]=BodyFKp[n];
        ControlBodyFKd[n]=BodyFKd[n];
        ControlBodyVKp[n]=BodyVKp[n];
        ControlQua_S[n]=Qua_S[n];
        ControlBodyAccMax[n]=BodyAccMax[n];
        ControlBodyAccMin[n]=BodyAccMin[n];
    }
    for(int n=0;n<3;n++)
    {
        ControlQua_W[n]=Qua_W[n];
        ControlInsideKp[n]=InsideKp[n];
        ControlInsideKd[n]=InsideKd[n];
        ControlSwingKp[n]=SwingKp[n];
        ControlSwingKd[n]=SwingKd[n];
    }
    //MathFun.Show(1,3,ControlSwingKd);
    return RIGHT;
}
int CJW_GaitToRobotBased::SetControlBodyFParameter(double BodyFKp[6],double BodyFKd[6])
{
    for(int n=0;n<6;n++)
    {
        ControlBodyFKp[n]=BodyFKp[n];
        ControlBodyFKd[n]=BodyFKd[n];
    }
    return RIGHT;
}
int CJW_GaitToRobotBased::PDController_F_BasedCOI(void)
{
    int error=0;
    /***********Body Controller based on SE(3) screw**********************************/
    struct CJW_JointStruct BodyExp,BodyReal;
    ExpRobot->GetMainBodyState(&BodyExp);
    RealRobot->GetMainBodyState(&BodyReal);
    double exp_acc[6]={0};for(int ii=0;ii<6;ii++){exp_acc[ii]=BodyExp.Body.BodyA[ii];}
    double control_acc[6]={0};MathFun.RigidBody_SE3_PDcontroller((double*)(BodyExp.G0),BodyExp.Body.BodyV,exp_acc,
                                                         (double*)(BodyReal.G0),BodyReal.Body.BodyV,ControlBodyFKp,ControlBodyFKd,control_acc);
    for(int ii=0;ii<6;ii++)//积分饱和
    {
        if(control_acc[ii]>ControlBodyAccMax[ii])
        {
            control_acc[ii]=ControlBodyAccMax[ii];
        }
        else if(control_acc[ii]<ControlBodyAccMin[ii])
        {
            control_acc[ii]=ControlBodyAccMin[ii];
        }
    }
    double COIForce[6]={0};RealRobot->GetRobotDynamicBodyForce(control_acc,COIForce);
    /***********supporting tip Controller based on quagramm**********************************/
    double BodyRefF[6]={0};double ExpTipF0[BRANCH_N_MAX*6]={0};
    error=RealRobot->GetExTipForceonQua(COIForce,ControlQua_S,ControlQua_W,BodyRefF,ExpTipF0);
    if(error) {printf("the acc is \n");MathFun.Show(1,6,control_acc);return error;}
    ExpRobot->SetMainBodyF(COIForce);
    /************control for every branchi*******************************************8*****************/
    struct CJW_TipStruct TipExp[BRANCH_N_MAX],TipReal[BRANCH_N_MAX];
    ExpRobot->GetMainModelTip(TipExp);RealRobot->GetMainModelTip(TipReal);
    double TipExpAbsV[BRANCH_N_MAX3];double TipRealAbsV[BRANCH_N_MAX3];
    int RobotBranchN=RealRobot->GetBranchN();
    /*************get the position velocity in the S coordinate**************/
    for(int legi=0;legi<RobotBranchN;legi++)
    {
        int legi3=legi*3;double theR[3][3];
        for(int ii=0;ii<3;ii++){for(int jj=0;jj<3;jj++){theR[ii][jj]=TipExp[legi].G[ii][jj];}}
        MathFun.MyRCompositionw((double*)theR,&(TipExp[legi].V[3]),&(TipExpAbsV[legi3]));
        for(int ii=0;ii<3;ii++){for(int jj=0;jj<3;jj++){theR[ii][jj]=TipReal[legi].G[ii][jj];}}
        MathFun.MyRCompositionw((double*)theR,&(TipReal[legi].V[3]),&(TipRealAbsV[legi3]));
    }
    /*************get PD controller swing and inside force in the S coordinate**************/
    double ExpTipF1[BRANCH_N_MAX3]={0};
    for(int legi=0;legi<RobotBranchN;legi++)
    {
        int legi3=legi*3;
        /***********supporting tip inside for different tips**********************************/
        //find all sup foot
        if(RealRobot->GetBranchiState(legi)>=BRANCH_BOUNDING_STATE)
        {
            for(int legj=legi+1;legj<RobotBranchN;legj++)
            {
                if(RealRobot->GetBranchiState(legj)>=BRANCH_BOUNDING_STATE)
                {
                    int legj3=legj*3;
                    for(int ii=0;ii<2;ii++)//compute XY plane
                    {
                        double inforce=ControlInsideKp[ii]*((TipReal[legi].G[ii][3]-TipReal[legj].G[ii][3])-(TipExp[legi].G[ii][3]-TipExp[legj].G[ii][3]))
                            +ControlInsideKd[ii]*((TipRealAbsV[legi3+ii]-TipRealAbsV[legj3+ii])-(TipExpAbsV[legi3+ii]-TipExpAbsV[legj3+ii]));
                        ExpTipF1[legi3+ii]=ExpTipF1[legi3+ii]+inforce;
                        ExpTipF1[legj3+ii]=ExpTipF1[legj3+ii]-inforce;
                    }
                }
            }
        }
        /***********swing tip Controller based on  PD controller**********************************/
        else
        {
            /****PD controller on the Tip for the  force*******/
            for(int ii=0;ii<3;ii++)
            {
                ExpTipF1[legi3+ii]=ControlSwingKp[ii]*(TipReal[legi].G[ii][3]-BodyReal.G0[ii][3]-TipExp[legi].G[ii][3]+BodyExp.G0[ii][3])
                        +ControlSwingKd[ii]*(TipRealAbsV[legi3+ii]-TipExpAbsV[legi3+ii]);

            }
        }
    }
    //printf("the tip f is %.3f+%.3f+%.3f\n",ExpTipF1[10],ExpTipF1[11],ExpTipF1[12]);
    /**********combine the swing leg and the supporting leg line force*******/
    for(int legi=0;legi<RobotBranchN;legi++)
    {
        int legi6=legi*6;int legi3=legi*3;
        double thetipR[9]={TipReal[legi].G[0][0],TipReal[legi].G[0][1],TipReal[legi].G[0][2],
                           TipReal[legi].G[1][0],TipReal[legi].G[1][1],TipReal[legi].G[1][2],
                           TipReal[legi].G[2][0],TipReal[legi].G[2][1],TipReal[legi].G[2][2]};
        double tempF1[6]={0};MathFun.MyRInvCompositionw(thetipR,&(ExpTipF1[legi3]),&(tempF1[3]));
        double outF[6]={0};for(int ii=0;ii<6;ii++){outF[ii]=-ExpTipF0[legi6+ii]-tempF1[ii];}
        //get the JointF using Jacobian;using line force without torque
        double theJacobain[6*BRANCH_BODY_MAX];
        int theJointN=RealRobot->BranchiModelGetJacobianTip(legi,theJacobain);
        double theJointF[BRANCH_BODY_MAX]={0};
        MathFun.MyMatrixMultiplyAT(theJointN,1,6,theJacobain,outF,theJointF);
        double theJointFG[BRANCH_BODY_MAX]={0};
        if(RealRobot->GetBranchiState(legi)<BRANCH_BOUNDING_STATE)//摆动重力补偿
        {
            RealRobot->BranchiModelGetGravityComForJoint(legi,theJointFG);
        }
        MathFun.MyMatrixAdd(theJointN,theJointF,theJointFG,theJointF);
        ExpRobot->SetBranchiModelAngleF(legi,theJointF);
        ExpRobot->SetBranchiModelTipF(legi,outF);
        ExpRobot->SetMainModelTipFOne(legi,outF);
        //printf("the leg %d TipF is\n",legi);MathFun.Show(1,6,outF);
        //printf("the leg %d Jocabin is\n",legi);MathFun.Show(6,theJointN,theJacobain);
        //printf("the leg %d angleF is\t",legi);MathFun.Show(1,theJointN,theJointF);
    }
    return error;
}
int CJW_GaitToRobotBased::PController_V_BasedTip(void)
{
    /***get the real and exp body position*******************************************/
    struct CJW_JointStruct BodyExp,BodyReal;
    ExpRobot->GetMainBodyState(&BodyExp);RealRobot->GetMainBodyState(&BodyReal);
    //double BodyExpGInv[16],BodyRealGInv[16];
    //MathFun.MyGInv((double*)BodyExp.G0,BodyExpGInv);MathFun.MyGInv((double*)BodyReal.G0,BodyRealGInv);
    /***control every tip*******************************************/
    int RobotBranchN=RealRobot->GetBranchN();
    for(int legi=0;legi<RobotBranchN;legi++)
    {
        struct CJW_TipStruct theTipAbsExp,theTipAbsReal;
        ExpRobot->GetMainModelTipOne(legi,&theTipAbsExp);
        RealRobot->GetMainModelTipOne(legi,&theTipAbsReal);
        double theTipBodyExp[4][4],theTipBodyReal[4][4];
        MathFun.MyGCompositionG((double*)(BodyExp.G0),(double*)(theTipAbsExp.G),(double*)theTipBodyExp);
        MathFun.MyGCompositionG((double*)(BodyReal.G0),(double*)(theTipAbsReal.G),(double*)theTipBodyReal);
        double dVBody[3]={0};
        for(int ii=0;ii<3;ii++){dVBody[ii]=ControlSwingKp[ii]*(theTipBodyExp[ii][3]-theTipBodyReal[ii][3]);}
        double theBodyRT[9]={theTipBodyExp[0][0],theTipBodyExp[1][0],theTipBodyExp[2][0],
                             theTipBodyExp[0][1],theTipBodyExp[1][1],theTipBodyExp[2][1],
                             theTipBodyExp[0][2],theTipBodyExp[1][2],theTipBodyExp[2][2]};
        double dVTip[3];MathFun.MyRCompositionw(theBodyRT,dVBody,dVTip);
        double AimTipV[6]={0,0,0,
                           dVTip[0]+theTipAbsExp.V[3],dVTip[1]+theTipAbsExp.V[4],dVTip[2]+theTipAbsExp.V[5]};
        //temp using the real robot to get joint velocity
        RealRobot->SetBranchiModelTipV(legi,AimTipV);
        RealRobot->BranchiModelTipVTransToAngleV(legi);
        double aim_anglev[BRANCH_BODY_MAX];RealRobot->GetBranchiModelAngleV(legi,aim_anglev);
        ExpRobot->SetBranchiModelAngleV(legi,aim_anglev);
    }
    return RIGHT;
}

int CJW_GaitToRobotBased::Control_Robot_Based(int ControlStype)
{
    int error=RIGHT;
    error=PDController_F_BasedCOI();
    if(error){error=GAIT_ERROR_INV_FORCE;}
    if (ControlStype == CONTROL_MODE_FORCE)
    {
        int RobotBranchN = RealRobot->GetBranchN();
        struct CJW_JointStruct BodyExp, BodyReal;
        ExpRobot->GetMainBodyState(&BodyExp);
        RealRobot->GetMainBodyState(&BodyReal);
        // /**************get the ref velocity********************************/
        ExpRobot->SetMainBodyG(BodyReal.G0);
        ExpRobot->SetMainBodyV(BodyReal.Body.BodyV);
        for (int legi = 0; legi < RobotBranchN; legi++)
        {
            if (RealRobot->GetBranchiState(legi) < BRANCH_BOUNDING_STATE)
            {
                ExpRobot->MainTipGTransToAnglePOne(legi);
                ExpRobot->MainTipVTransToAngleVOne(legi);
            }
        }
#ifdef CONTROL_VELOCITY_PRE
        double control_vel[6] = {0};
        MathFun.RigidBody_SE3_Pcontroller((double *)(BodyExp.G0), BodyExp.Body.BodyV,
                                          (double *)(BodyReal.G0), ControlBodyVKp, control_vel);
        //printf("the velocity is \n");MathFun.Show(1,6,control_vel);
        ExpRobot->SetMainBodyV(control_vel);
        for (int legi = 0; legi < RobotBranchN; legi++)
        {
            if (RealRobot->GetBranchiState(legi) >= BRANCH_BOUNDING_STATE)
            {
                ExpRobot->MainTipGTransToAnglePOne(legi);
                ExpRobot->MainTipVTransToAngleVOne(legi);
            }
        }
#endif
        ExpRobot->SetMainBodyV(BodyExp.Body.BodyV);
        ExpRobot->SetMainBodyG(BodyExp.G0);
    }
    else if(ControlStype==CONTROL_MODE_VELOCITY)
    {
        //error=PController_V_BasedTip();
    } 
    else
    {

    }
    return error;
}

/*************************************************************************************************************/
/*************************************************************************************************************/
/*******************************gait of the robot************************************************************************/
int CJW_GaitToRobotBased::SetInitState(void)
{
    if(RealRobot->GetBranchN()!=ThisGait->GetGaitN()) {return ERROR;}
    RealRobot->GetMainBodyState(&GaitInitBody0);
    RealRobot->GetMainModelTip(GaitInitTip0);
    RealRobot->GetALLAngleState(GaitInitJoint0);
    Design_Robot_MapToInital(&(Gait_Body),Gait_Tip,&Gait_BodySE2,Gait_TipSE2);
    return RIGHT;
}
int CJW_GaitToRobotBased::SetInitState(struct CJW_LWARobot* ThisRobot)
{
    if(ThisRobot->GetBranchN()!=ThisGait->GetGaitN()) {return ERROR;}
    ThisRobot->GetMainBodyState(&GaitInitBody0);
    ThisRobot->GetMainModelTip(GaitInitTip0);
    ThisRobot->GetALLAngleState(GaitInitJoint0);
    return RIGHT;
}
int CJW_GaitToRobotBased::SetInitState(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0)
{
    if(RealRobot->GetBranchN()!=ThisGait->GetGaitN()) {return ERROR;}
    GaitInitBody0=Body0[0];
    for(int legi;legi<RealRobot->GetBranchN();legi++)
    {
        GaitInitTip0[legi]=Tip0[legi];
    }
    return RIGHT;
}
int CJW_GaitToRobotBased::SetGaitMotionType(int gaittype,int movemode,int bodytype,int swingtype)
{
    GaitType=gaittype;
    MoveMode=movemode;
    BodyType=bodytype;
    SwingType=swingtype;
    return RIGHT;
}
int CJW_GaitToRobotBased::SetGaitParameter(struct CJW_JointStruct *Body0,struct CJW_TipStruct* Tip0, double T0,double stepH,double* dir,double* margin)
{
    Gait_Body=Body0[0];
    for(int legi;legi<RealRobot->GetBranchN();legi++)
    {
        Gait_Tip[legi]=Tip0[legi];
    }
    Gait_T0=T0;
    Gait_StepH=stepH;
    for(int ii=0;ii<3;ii++)
    {
        Gait_Dir[ii]=dir[ii];
    }
    for(int ii=0;ii<5;ii++)
    {
        Gait_Margin[ii]=margin[ii];
    }
    return RIGHT;
}
int CJW_GaitToRobotBased::GetContactFlag(void)
{
    int swingn=0;
    for(int branchi=0;branchi<ThisGait->GetGaitN();branchi++)
    {
        swingn=swingn+GaitSwingflag[branchi];
    }
    return swingn;
}
/******************gait for based on user************************************************************/
//stop move
int CJW_GaitToRobotBased::Design_Robot_Emergency(void)
{
    RealRobot->GetALLAngleState(GaitInitJoint0);
    //emergency robot
    for(int jointi=0;jointi<(BRANCH_N_MAX*BRANCH_BODY_MAX);jointi++)
    {
        GaitInitJoint0[jointi].Velocity=0;
        GaitInitJoint0[jointi].Accelation=0;
        GaitInitJoint0[jointi].Force=0;
    }
    for(int branchi=0;branchi<BRANCH_N_MAX;branchi++)
    {
        ExpWheelDir[branchi]=0;
        ExpWheelSpeed[branchi]=0;
    }
    ExpRobot->SetALLAngleState(GaitInitJoint0);
    return RIGHT;
}
/******************stop move for LAW_robot*****************************************************/
int CJW_GaitToRobotBased::Design_Robot_Stop(void)
{
    double theN=RealRobot->GetBranchN();
    double dataZERO[6]={0};
    ExpRobot->SetMainBodyV(dataZERO);
    ExpRobot->SetMainBodyA(dataZERO);
    int model0=RealRobot->GetBranchiModel(0);
    for(int i=0;i<theN;i++)
    {
        ExpRobot->SetMainModelTipVOne(i,dataZERO);
        ExpRobot->SetMainModelTipAOne(i,dataZERO);
        int thismodel=RealRobot->GetBranchiModel(i);
        ExpRobot->SetBranchiModel(i,thismodel);
        ThisGait->SetBranchiModel(i,thismodel);
        if(thismodel==BRANCH_WHEEL_MODEL)
        {
            ExpWheelDir[i]=0; 
        }
        else
        {
            ExpWheelDir[i]=0; 
        }
        if(thismodel!=model0){model0=ROBOT_MODEL_MIX;}
        int thisstate=ExpRobot->GetBranchiState(i)>=BRANCH_BOUNDING_STATE;
        ExpRobot->SetBranchiState(i,thisstate);
    }
    ExpRobot->SetRobotModel(model0);
    RealRobot->SetRobotModel(model0);
    return RIGHT;
}
int CJW_GaitToRobotBased::Design_Robot_MapToInital(struct CJW_JointStruct *Body0,struct CJW_TipStruct* Tip0,
                                                    struct CJW_JointStruct *Bodyout,struct CJW_TipStruct* Tipout)
{
    double G0_SE2[4][4];MathFun.MySE3ToSE2((double*)GaitInitBody0.G0,(double*)G0_SE2);
    Bodyout[0]=Body0[0];
    MathFun.MyGCompositionG((double*)G0_SE2,(double*)Body0->G0,(double*)Bodyout->G0);
    double theN=ExpRobot->GetBranchN();
    for(int i=0;i<theN;i++)
    {
        Tipout[i]=Tip0[i];
        MathFun.MyGCompositionG((double*)G0_SE2,(double*)(Tip0[i].G),(double*)(Tipout[i].G));
    }
    return RIGHT; 
}
//set the exp robot body GVA in body coordinate and the Tip G V A to the joint position and velocity
//sepcial in the LEG and WHEEL model ,the Tip velcoity and accelation is the spcecial coordiante
int CJW_GaitToRobotBased::Design_BodyTipToJoint(struct CJW_JointStruct *Body,struct CJW_TipStruct* Tip,double* BranchiState,int* BranchiType)
{
    //test the branchiN and model
    double theN=ExpRobot->GetBranchN();if(theN!=ThisGait->GetGaitN()) {return GAIT_ERROR_ROBOT_STATE;}
    for(int legi=0;legi<theN;legi++)
    {
        ExpRobot->SetBranchiState(legi,BranchiState[legi]);
        //RealRobot->SetBranchiState(legi,BranchiState[legi]);
        if(ExpRobot->GetBranchiModel(legi)!=(ThisGait->GetBranchiModel(legi))){return GAIT_ERROR_LEG_STATE;}
        //wheel dir in leg model is set zero
        if(ExpRobot->GetBranchiModel(legi)!=BRANCH_WHEEL_MODEL)
        {
            ExpWheelDir[legi]=0;//atan2(Body->G0[1][0], Body->G0[0][0]);
        }
    }
    //set position angle
    ExpRobot->SetMainBodyState(Body);
    //set tip G To the angle position
    double BodySE2[4][4];
    MathFun.MySE3ToSE2((double *)Body->G0, (double *)BodySE2);
    for(int legi=0;legi<theN;legi++)
    {
        if((ExpRobot->GetBranchiModel(legi)==BRANCH_WHEEL_MODEL)&&(BranchiType[legi]==TIP_DATA_SPE))
        {//using the numerical inverted kinematics with wheel geometry
            Tip[legi].G[0][0]=BodySE2[0][0]*cos(ExpWheelDir[legi])+BodySE2[0][1]*sin(ExpWheelDir[legi]);
            Tip[legi].G[1][0]=BodySE2[1][0]*cos(ExpWheelDir[legi])+BodySE2[1][1]*sin(ExpWheelDir[legi]);
            Tip[legi].G[0][1]=-BodySE2[0][0]*sin(ExpWheelDir[legi])+BodySE2[0][1]*cos(ExpWheelDir[legi]);
            Tip[legi].G[1][1]=-BodySE2[1][0]*sin(ExpWheelDir[legi])+BodySE2[1][1]*cos(ExpWheelDir[legi]);
            Tip[legi].G[0][2]=0;Tip[legi].G[1][2]=0;
            Tip[legi].G[2][0]=0;Tip[legi].G[2][1]=0;
            Tip[legi].G[2][2]=1;
        }
        ExpRobot->SetMainModelTipGOne(legi,Tip[legi].G);
    }
    int error=RIGHT;
    error=ExpRobot->MainTipGTransToAngleP();
    if(error){return GAIT_ERROR_INV_POSITION;}
    //set Tip V and A To the angle velocity accelation
    for(int legi=0;legi<theN;legi++)
    {
        if(BranchiType[legi]==TIP_DATA_SPE)
        {//because the foot is in the special coordinate (orietation is parallel to the ground)
            double theG[4][4];ExpRobot->GetMainModelTipGOne(legi,theG);
            double theRT[9];double theP[3];MathFun.MyGToRInvP((double *)theG,theRT,theP);
            double TipVV[6]={0};double TipAA[6]={0};
            MathFun.MyRCompositionw(theRT,&(Tip[legi].V[3]),&(TipVV[3]));
            MathFun.MyRCompositionw(theRT,&(Tip[legi].A[3]),&(TipAA[3]));
            ExpRobot->SetMainModelTipVOne(legi,TipVV);
            ExpRobot->SetMainModelTipAOne(legi,TipAA);
        }
        else
        {
            ExpRobot->SetMainModelTipVOne(legi,Tip[legi].V);
            ExpRobot->SetMainModelTipAOne(legi,Tip[legi].A);
        }
    }
    error=10*ExpRobot->MainTipVTransToAngleV();
    if(error){return GAIT_ERROR_INV_VELOCITY;}

    /*******************wheel design**********************/
    ExpRobot->SetAllWheelDirection(ExpWheelDir);
    ExpRobot->SetAllWheelSpeed(ExpWheelSpeed);

    return error;
}

/******************tip move for LAW_robot*****************************************************/
int CJW_GaitToRobotBased::Design_MainTip_PPmode(double* State,double time)
{
    ComBody=GaitInitBody0;
    ThisGait->Design_SE3_ZERO(DESIGN_MOVE_DECOUPE,DESIGN_QUINTIC,GaitInitBody0.G0,Gait_BodySE2.G0,Gait_T0,time,ComBody.G0,ComBody.Body.BodyV,ComBody.Body.BodyA);
    for(int legi=0;legi<ThisGait->GetGaitN();legi++)
    {
        ComTip[legi]=GaitInitTip0[legi];
        int Branchimodel=ExpRobot->GetBranchiModel(legi);
        if((Branchimodel==BRANCH_LEG_MODEL)||(Branchimodel==BRANCH_WHEEL_MODEL))
        {
            double theR1[3][3];double theP1[3];MathFun.MyGToRP((double*)(GaitInitTip0[legi].G),(double*)theR1,theP1);
            double theR2[3][3];double theP2[3];MathFun.MyGToRP((double*)(Gait_TipSE2[legi].G),(double*)theR2,theP2);
            double outR[3][3];double outP[3];
            ThisGait->Design_SO3_ZERO(DESIGN_QUINTIC,theR1,theR2,Gait_T0,time,outR,ComTip[legi].V,ComTip[legi].A);
            ThisGait->Design_R3_ZERO(DESIGN_QUINTIC,theP1,theP2,Gait_T0,time,outP,&(ComTip[legi].V[3]),&(ComTip[legi].A[3]));
            MathFun.MyRPToG((double*)outR,outP,(double*)(ComTip[legi].G));
            ComBraType[legi]=TIP_DATA_SPE;
        }
        else
        {
            ThisGait->Design_SE3_ZERO(DESIGN_MOVE_DECOUPE,DESIGN_QUINTIC,GaitInitTip0[legi].G,Gait_TipSE2[legi].G,Gait_T0,time,ComTip[legi].G,ComTip[legi].V,ComTip[legi].A);
            ComBraType[legi]=TIP_DATA_SELF;
        }
    }
    int error=Design_BodyTipToJoint(&ComBody,ComTip,State,ComBraType);
    return error;
}
int CJW_GaitToRobotBased::Design_MainTip_P0Test(double* State,double Amp[6],double time)
{
    ComBody=GaitInitBody0;
    double EnddG1[4][4];MathFun.MyExponent4ToG(Amp,(double*)EnddG1);
    double EndG1[4][4];MathFun.MyGCompositionG((double*)Gait_BodySE2.G0,(double*)EnddG1,(double*)EndG1);
    if(time<(Gait_T0/2))
    {
        ThisGait->Design_SE3_ZERO(DESIGN_MOVE_DECOUPE,DESIGN_CUBIC,Gait_BodySE2.G0,EndG1,Gait_T0/2,time,ComBody.G0,ComBody.Body.BodyV,ComBody.Body.BodyA);
    }
    else
    {
        ThisGait->Design_SE3_ZERO(DESIGN_MOVE_DECOUPE,DESIGN_CUBIC,EndG1,Gait_BodySE2.G0,Gait_T0/2,(time-Gait_T0/2),ComBody.G0,ComBody.Body.BodyV,ComBody.Body.BodyA);
    }
    for(int legi=0;legi<ThisGait->GetGaitN();legi++)
    {
        ComTip[legi]=Gait_TipSE2[legi];
        ComBraType[legi]=TIP_DATA_SPE;
    }
    int error=Design_BodyTipToJoint(&ComBody,ComTip,State,ComBraType);
    return error;
}

/*********************Auto choose the gait****************************************/
int CJW_GaitToRobotBased::Design_Move_Refresh(double MAX_A_SE2[6])
{
    int Robotmodel=RealRobot->GetRobotModel();
    if(Robotmodel==ROBOT_MODEL_LEG)
    {
        if((GaitType==QUADRUPED_GAIT_PACE)||(GaitType==QUADRUPED_GAIT_TROT))
        {
            Design_Walk_Adapt_Refresh(MAX_A_SE2);
        }
        else if(GaitType==QUADRUPED_GAIT_BOUNDING)
        {

        }
        else if(GaitType==QUADRUPED_GAIT_STATIC)
        {

        }
        else
        {

        }
    }
    else if(Robotmodel==ROBOT_MODEL_WHEEL)
    {
        if((GaitType==QUADRUPED_GAIT_PACE)||(GaitType==QUADRUPED_GAIT_TROT))
        {
            Design_Skate_Adapt_Refresh();
        }
        else if(GaitType==QUADRUPED_GAIT_BOUNDING)
        {

        }
        else if(GaitType==QUADRUPED_GAIT_STATIC)
        {
            Design_Skate_Static_Refresh();
        }
        else if(GaitType==QUADRUPED_GAIT_SWIZZLING)
        {
            Design_Skate_Swizzling_Refresh();
        }
        else
        {

        }
    }
    else
    {

    }
    return RIGHT;
}
int CJW_GaitToRobotBased::Design_Move_Run(int StartFlag,double time)
{
    int Robotmodel=RealRobot->GetRobotModel();
    if(Robotmodel==ROBOT_MODEL_LEG)
    {
        if((GaitType==QUADRUPED_GAIT_PACE)||(GaitType==QUADRUPED_GAIT_TROT))
        {
            Design_Walk_Adapt_Based(time);
            //Design_LegMove_Kinetic(time);
        }
        else if(GaitType==QUADRUPED_GAIT_STATIC)
        {
            
        }
        else
        {

        }
    }
    else if(Robotmodel==ROBOT_MODEL_WHEEL)
    {
        if((GaitType==QUADRUPED_GAIT_PACE)||(GaitType==QUADRUPED_GAIT_TROT))
        {
            Design_Skate_Adapt_Based(time);
            //Design_Skating_Kinetic(time);
        }
        else if(GaitType==QUADRUPED_GAIT_STATIC)
        {
            Design_Skate_Static_Based(time);
        }
        else if(GaitType==QUADRUPED_GAIT_SWIZZLING)
        {
            Design_Skate_Swizzling_Based(StartFlag,time);
        }
        else
        {

        }
    }
    else
    {

    }
    return RIGHT;
}




/******************open gait *****************************************************************************/
/******************legged gait for LAW_robot*****************************************************/
int CJW_GaitToRobotBased::Design_LegMove_Kinetic(double time)
{
    //if((ExpRobot->GetRobotModel())!=ROBOT_MODEL_LEG) {return -ERROR;}
    ThisGait->Design_Leg_Position0(&Gait_BodySE2,Gait_TipSE2,Gait_Dir,Gait_StepH,Gait_T0,time,MoveMode,BodyType,GaitType,SwingType,&ComBody,ComTip,ComBraState);
    for(int legi=0;legi<ThisGait->GetGaitN();legi++)
    {
        ComBraType[legi]=TIP_DATA_SPE;
    }
    int error=Design_BodyTipToJoint(&ComBody,ComTip,ComBraState,ComBraType);
    return error;
}
/******************trans bewteen legged and wheel for LAW_robot*****************************************************/
int CJW_GaitToRobotBased::Design_BetweenLegAndWheel_Kinetic(struct CJW_JointStruct *BodyMid,struct CJW_TipStruct* TipMid,
                                                            int* leg_model,double time)
{
    int TheN=ThisGait->GetGaitN();
    struct CJW_JointStruct BodyMid0=*BodyMid;struct CJW_TipStruct TipMid0[BRANCH_N_MAX];
    Design_Robot_MapToInital(BodyMid,TipMid,&BodyMid0,TipMid0);
    int thismode[BRANCH_N_MAX]={};
    for(int legi=0;legi<TheN;legi++)
    {
        thismode[legi]=RealRobot->GetBranchiModel(legi);
    }
    double theT0=1.0*Gait_T0/2;
    struct CJW_JointStruct* BodyA;struct CJW_TipStruct* TipA;
    struct CJW_JointStruct* BodyB;struct CJW_TipStruct* TipB;
    double thetime=time;
    if(time<=theT0)
    {
        BodyA=&(GaitInitBody0);TipA=GaitInitTip0;
        BodyB=&BodyMid0;TipB=TipMid0;
    }
    else
    {
        thetime = time - theT0;
        int model0=leg_model[0];
        for (int legi = 0; legi < TheN; legi++)
        {
            thismode[legi] = leg_model[legi];
            if(leg_model[legi]!=model0){model0=ROBOT_MODEL_MIX;}
            ExpRobot->SetBranchiModel(legi,thismode[legi]);
            ThisGait->SetBranchiModel(legi,thismode[legi]);
        }
        ExpRobot->SetRobotModel(model0);
        BodyA = &BodyMid0;TipA = TipMid0;
        BodyB = &Gait_BodySE2;TipB = Gait_TipSE2;
    }
    ComBody=GaitInitBody0;
    ThisGait->Design_SE3_ZERO(DESIGN_MOVE_DECOUPE,DESIGN_CUBIC,BodyA->G0,BodyB->G0,theT0,thetime,ComBody.G0,ComBody.Body.BodyV,ComBody.Body.BodyA);
    for(int legi=0;legi<TheN;legi++)
    {
        double theR1[3][3];double theP1[3];MathFun.MyGToRP((double*)(TipA[legi].G),(double*)theR1,theP1);
        double theR2[3][3];double theP2[3];MathFun.MyGToRP((double*)(TipB[legi].G),(double*)theR2,theP2);
        double outR[3][3];double outP[3];
        ThisGait->Design_SO3_ZERO(DESIGN_SIN,theR1,theR2,theT0,thetime,outR,ComTip[legi].V,ComTip[legi].A);
        ThisGait->Design_R3_ZERO(DESIGN_SIN,theP1,theP2,theT0,thetime,outP,&(ComTip[legi].V[3]),&(ComTip[legi].A[3]));
        MathFun.MyRPToG((double*)outR,outP,(double*)(ComTip[legi].G));
        ComBraType[legi]=TIP_DATA_SPE;
        ComBraState[legi]=BRANCH_SUP_STATE;
    }
    int error=Design_BodyTipToJoint(&ComBody,ComTip,ComBraState,ComBraType);
    return error;
}
/******************skating gait for LAW_robot*****************************************************/
int CJW_GaitToRobotBased::Design_Skating_Kinetic(double time)
{
    if((ExpRobot->GetRobotModel())!=ROBOT_MODEL_WHEEL) {return -ERROR;}
    double Step[2]={Gait_StepH,Gait_Dir[1]};
    double duty=Gait_Margin[2];
    if((duty<=0.05)||(duty>=0.95)){duty=0.5;}
    ThisGait->Design_Skating_Position0(&Gait_BodySE2,Gait_TipSE2,Gait_Margin,Step,Gait_T0,duty,time,MoveMode,DESIGN_QUINTIC,GaitType,SwingType,&ComBody,ComTip,ComBraState);
    double Wheel_dir_dir[BRANCH_N_MAX]={0};
    ThisGait->GetWheelDirection(GaitType,Wheel_dir_dir);
    for(int legi=0;legi<ThisGait->GetGaitN();legi++)
    {
        ComBraType[legi]=TIP_DATA_SPE;
        ExpWheelDir[legi]=Wheel_dir_dir[legi]*Gait_Margin[4];  
    }
    int error=Design_BodyTipToJoint(&ComBody,ComTip,ComBraState,ComBraType);
    return error;
}
/*********************bounding gait**************************************************/
//outT: TF,TH,dT
//outBody: pz,thy,vz,wy,az,ay,alltime
//outFoot: pf,vf,ph,vh
int CJW_GaitToRobotBased::Design_DoubleSLIP_Kinetic(double Zmax, double TJ, double ZJ, double thmax, double LB,
                              double time0,double* outT,double* outBody)
{
    //double Zmax=BOUNDING_Z_MAX;
    double Zmin=BOUNDING_Z_MIN;
    double thJ=-fabs(thmax);double vJ=TJ*WORLD_GRAVITY;double wJ=fabs(thJ/TJ);
    //compute double SLIP parameters
    double zH0=ZJ+LB*sin(thJ);
    double AH=zH0-Zmin;
    double wh=(vJ+LB*wJ*cos(thJ))/AH;double TH=M_PI/2/wh;
    double zF0=ZJ-LB*sin(thJ);
    if(zF0>Zmax){zF0=Zmax;}
    double AF=zF0-Zmin;double vJf=vJ-LB*wJ*cos(thJ);
    double wf=sqrt(vJf*vJf+2*WORLD_GRAVITY*(ZJ-LB*sin(thJ)-zF0))/AF;
    double TF=M_PI/2/wf;
    double dT=(wf*AF-vJf)/WORLD_GRAVITY+TF-TH;
    outT[0]=TF;
    outT[1]=TH;
    outT[2]=dT;
    //compute the double SLIP model
    double zf=zF0-AF;double vf=0;double af=0;
    double zh=zH0-AH;double vh=0;double ah=0;
    double time=time0;
    if(time>=(dT+TH))
    {
        time=dT+TH;
    }
    if(time<=TF)
    {
        zf=zF0-AF*cos(wf*time);
        vf=AF*sin(wf*time)*wf;
        af=AF*cos(wf*time)*wf*wf;
    }
    else
    {
        double thet=time-TF;
        zf=zF0+AF*wf*thet-0.5*WORLD_GRAVITY*thet*thet;
        vf=AF*wf-WORLD_GRAVITY*thet;
        af=-WORLD_GRAVITY;
    }
    if(time>=dT)
    {
        double thet=time-dT;
        zh=zH0-AH*cos(wh*thet);
        vh=AH*sin(wh*thet)*wh;
        ah=AH*cos(wh*thet)*wh*wh;
    }
    //transform to the z and thy
    outBody[0]=(zf+zh)/2;
    outBody[2]=(vf+vh)/2;
    outBody[4]=(af+ah)/2;
    outBody[1]=asin((zh-zf)/2/LB);
    outBody[3]=(vh-vf)/cos(outBody[1])/2/LB;
    outBody[5]=(ah-af)/cos(outBody[1])/2/LB+outBody[3]*outBody[3]*tan(outBody[1]);
    
    return (time>=(dT+TH));
}
//para: TJ,ZJ,thmax
int CJW_GaitToRobotBased::Design_Bounding_Kinetic(double time,int *flag)
{   
    double para[3]={Gait_T0,Gait_Dir[1],Gait_Dir[2]};
    ////////////////////////design in the XZ plane
    struct CJW_JointStruct BodyMap=Gait_Body;struct CJW_TipStruct TipMap[BRANCH_N_MAX];
    for(int legi=0;legi<ThisGait->GetGaitN();legi++)
    { 
        TipMap[legi]=Gait_Tip[legi];
        ComBraState[legi]=BRANCH_SUP_STATE;
        ComBraType[legi]=TIP_DATA_SPE;
        ExpWheelDir[legi]=0;
    }
    double S_Zmax=Gait_Body.G0[2][3];
    if(S_Zmax<=BOUNDING_Z_MIN){S_Zmax=BOUNDING_Z_MAX;}
    double theLB=fabs(Gait_Tip[0].G[0][3]-Gait_Tip[3].G[0][3])/2;
    double S_TJ=para[0];double S_thmax=para[2];
    double S_ZJ=theLB*sin(fabs(S_thmax))+S_Zmax;
    double outBody[6]={0};double outT0[3]={0};
    Design_DoubleSLIP_Kinetic(S_Zmax,S_TJ,S_ZJ,S_thmax,theLB,0,outT0,outBody);
    double all_time=outT0[1]+outT0[2];
    //机身规划
    if(time<all_time)
    {
        Design_DoubleSLIP_Kinetic(S_Zmax,S_TJ,S_ZJ,S_thmax,theLB,time,outT0,outBody);
    }
    else if(time<(all_time+2*S_TJ))
    {
        double thet=all_time+S_TJ-time;
        outBody[0]=S_ZJ+0.5*WORLD_GRAVITY*(S_TJ*S_TJ-thet*thet);
        outBody[2]=WORLD_GRAVITY*thet;
        outBody[4]=-WORLD_GRAVITY;
        outBody[1]=-fabs(S_thmax)*thet/S_TJ;
        outBody[3]=fabs(S_thmax)/S_TJ;
        outBody[5]=0;
    }
    else if(time<(2*all_time+2*S_TJ))
    {
        double thet=2*all_time+2*S_TJ-time;
        Design_DoubleSLIP_Kinetic(S_Zmax,S_TJ,S_ZJ,S_thmax,theLB,thet,outT0,outBody);
        outBody[1]=-outBody[1];
        outBody[2]=-outBody[2];
    }
    BodyMap.G0[0][0]=cos(outBody[1]);BodyMap.G0[0][1]=0;BodyMap.G0[0][2]=sin(outBody[1]);
    BodyMap.G0[1][0]=0;BodyMap.G0[1][1]=1;BodyMap.G0[1][2]=0;
    BodyMap.G0[2][0]=-sin(outBody[1]);BodyMap.G0[2][1]=0;BodyMap.G0[2][2]=cos(outBody[1]);
    BodyMap.Body.BodyV[1]=outBody[3];
    BodyMap.Body.BodyA[1]=outBody[5];
    BodyMap.G0[2][3]=outBody[0];
    BodyMap.Body.BodyV[3]=-sin(outBody[1])*outBody[2];
    BodyMap.Body.BodyV[5]=cos(outBody[1])*outBody[2];
    BodyMap.Body.BodyA[3]=-sin(outBody[1])*outBody[4];
    BodyMap.Body.BodyA[5]=cos(outBody[1])*outBody[4];
    //单腿规划规划
    double T1[2]={outT0[0],outT0[1]+outT0[2]};
    double T2[2]={all_time+2*S_TJ,2*all_time+2*S_TJ-outT0[0]};
    double swingT0=BOUNDING_SWING_T0;
    for(int thei=0;thei<2;thei++)
    {
        int thej=1-2*thei;
        int theii=thei+1;
        //double foot_LB=thej*theLB;
        double foot_LB=Gait_Tip[theii].G[0][3];
        //along the Z axis
        if(time<=T1[thei])
        {
            TipMap[theii].G[2][3]=0;TipMap[theii].V[5]=0;
        }
        else if(time<(T1[thei]+swingT0))
        {
            double thebody[6]={0};
            if(T1[thei]+swingT0<all_time)
            {
                Design_DoubleSLIP_Kinetic(S_Zmax,S_TJ,S_ZJ,S_thmax,theLB,(T1[thei]+swingT0),outT0,thebody);
            }
            else
            {
                double thet=all_time+S_TJ-(T1[thei]+swingT0);
                thebody[0]=S_ZJ+0.5*WORLD_GRAVITY*(S_TJ*S_TJ-thet*thet);
                thebody[2]=WORLD_GRAVITY*thet;
                thebody[1]=-fabs(S_thmax)*thet/S_TJ;
                thebody[3]=fabs(S_thmax)/S_TJ;
            }
            double foot_start[3]={0};
            double foot_end[3]={thebody[0]-sin(thebody[1])*foot_LB-BOUNDING_Z_MIN,thebody[2]-cos(thebody[1])*foot_LB*thebody[3],0};
            double foot_now[3]={0};
            ThisGait->Design_1DoF(DESIGN_CUBIC,foot_start,foot_end,swingT0,time-T1[thei],foot_now);
            TipMap[theii].G[2][3]=foot_now[0];TipMap[theii].V[5]=foot_now[1];
            ComBraState[theii]=BRANCH_SWI_STATE;
        }
        else if(time<=(T2[thei]-swingT0))
        {
            TipMap[theii].G[2][3]=outBody[0]-sin(outBody[1])*foot_LB-BOUNDING_Z_MIN;
            TipMap[theii].V[5]=outBody[2]-cos(outBody[1])*foot_LB*outBody[3];
            ComBraState[theii]=BRANCH_SWI_STATE;
        }
        else if(time<T2[thei])
        {
            double thebody[6]={0};
            if((2*all_time+2*S_TJ-T2[thei]+swingT0)<all_time)
            {
                Design_DoubleSLIP_Kinetic(S_Zmax,S_TJ,S_ZJ,S_thmax,theLB,(2*all_time+2*S_TJ-T2[thei]+swingT0),outT0,thebody);
                thebody[1]=-thebody[1];
                thebody[2]=-thebody[2];
            }
            else
            {
                double thet=all_time+S_TJ-(T2[thei]-swingT0);
                thebody[0]=S_ZJ+0.5*WORLD_GRAVITY*(S_TJ*S_TJ-thet*thet);
                thebody[2]=WORLD_GRAVITY*thet;
                thebody[1]=-fabs(S_thmax)*thet/S_TJ;
                thebody[3]=fabs(S_thmax)/S_TJ;
            }     
            double foot_start[3]={0};
            double foot_end[3]={thebody[0]-sin(thebody[1])*foot_LB-BOUNDING_Z_MIN,(thebody[2]-cos(thebody[1])*foot_LB*thebody[3]),0};
            double foot_now[3]={0};
            ThisGait->Design_1DoF(DESIGN_CUBIC,foot_end,foot_start,swingT0,time-T2[thei]+swingT0,foot_now);
            TipMap[theii].G[2][3]=foot_now[0];TipMap[theii].V[5]=foot_now[1];
            ComBraState[theii]=BRANCH_SWI_STATE;
        }
        else
        {
            TipMap[theii].G[2][3]=0;TipMap[theii].V[5]=0;
        }
        //along X axis
        TipMap[theii].G[0][3]=foot_LB*cos(outBody[1]);
        TipMap[theii].V[3]=foot_LB*sin(outBody[1])*outBody[3];
    }
    TipMap[0].G[0][3]=TipMap[1].G[0][3];
    TipMap[0].G[2][3]=TipMap[1].G[2][3];
    TipMap[0].V[3]=TipMap[1].V[3];
    TipMap[0].V[5]=TipMap[1].V[5];
    TipMap[3].G[0][3]=TipMap[2].G[0][3];
    TipMap[3].G[2][3]=TipMap[2].G[2][3];
    TipMap[3].V[3]=TipMap[2].V[3];
    TipMap[3].V[5]=TipMap[2].V[5];
    ComBraState[0]=ComBraState[1];
    ComBraState[3]=ComBraState[2];

    //////////////////////Map to the absolute
    ComBody=GaitInitBody0;
    Design_Robot_MapToInital(&BodyMap,TipMap,&ComBody,ComTip);
    flag[0]=time>=(all_time*2+S_TJ*2);
    if(flag[0]){printf("Bounding: TJ=%.3f;TF=%.3f;TH=%.3f;dT=%.3f\n",S_TJ,outT0[0],outT0[1],outT0[2]);}
    int error=Design_BodyTipToJoint(&ComBody,ComTip,ComBraState,ComBraType);
    return error;   
}


/******************adaptive  gait *****************************************************************************/
/******************adaptive walking gait *****************************************************************************/
int CJW_GaitToRobotBased::Design_Walk_Adapt_Fore(struct CJW_JointStruct *BodyZ,struct CJW_JointStruct *BodyNow,double MAX_A_SE2[6],double TT,struct CJW_JointStruct *BodyNext)
{
    //trans to the absolute velocity
    double BodyWPS[6]={BodyNow->Body.BodyV[0],BodyNow->Body.BodyV[1],BodyNow->Body.BodyV[2],
                      BodyNow->Body.BodyV[3],BodyNow->Body.BodyV[4],BodyNow->Body.BodyV[5]};
    double BodyR[9];double BodyP[3];MathFun.MyGToRP((double*)(BodyNow->G0),BodyR,BodyP);
    MathFun.MyRCompositionw(BodyR,BodyNow->Body.BodyV,BodyWPS);
    MathFun.MyRCompositionw(BodyR,&(BodyNow->Body.BodyV[3]),&(BodyWPS[3]));
    //To the Now direction
    double BodyRZYX[3];MathFun.MyRToEulerZYX(BodyR,BodyRZYX);
    double tempvxy[2]={BodyWPS[3]*cos(BodyRZYX[0])+BodyWPS[4]*sin(BodyRZYX[0]),
                      -BodyWPS[3]*sin(BodyRZYX[0])+BodyWPS[4]*cos(BodyRZYX[0])};
    BodyWPS[3]=tempvxy[0];BodyWPS[4]=tempvxy[1];
    //predict the feature body position and velocity
    double ForeRP[6]={0};
    for(int ii=0;ii<6;ii++)
    {
        BodyNext->Body.BodyV[ii]=0;
        BodyNext->Body.BodyA[ii]=0;
    }
    for(int ii=2;ii<5;ii++)
    {
        double dis1=BodyZ->Body.BodyV[ii]-BodyWPS[ii];
        if(fabs(dis1)>(MAX_A_SE2[ii]*TT))
        {
            BodyNext->Body.BodyA[ii]=dis1/fabs(dis1)*MAX_A_SE2[ii];
            BodyNext->Body.BodyV[ii]=BodyWPS[ii]+dis1/fabs(dis1)*MAX_A_SE2[ii]*TT;
        }
        else
        {
            BodyNext->Body.BodyA[ii]=dis1/TT;
            BodyNext->Body.BodyV[ii]=BodyZ->Body.BodyV[ii];
        }
        ForeRP[ii]=(BodyWPS[ii]+BodyNext->Body.BodyV[ii])/2*TT;
    }
    double ForeG[4][4]={cos(ForeRP[2]),-sin(ForeRP[2]),0,ForeRP[3],
                        sin(ForeRP[2]),cos(ForeRP[2]),0,ForeRP[4],
                        0,0,1,0,
                        0,0,0,1};
    //compute the absolute expect
    double BodyGZ0[16]={cos(BodyRZYX[0]),-sin(BodyRZYX[0]),0,BodyNow->G0[0][3],
                       sin(BodyRZYX[0]),cos(BodyRZYX[0]),0,  BodyNow->G0[1][3],
                       0,0,1,                                BodyZ->G0[2][3],
                       0,0,0,1};
    MathFun.MyGCompositionG((double*)BodyGZ0,(double*)ForeG,(double*)(BodyNext->G0));
    return RIGHT;
}

int CJW_GaitToRobotBased::Design_Walk_Adapt_Refresh(double MAX_A_SE2[6])
{
    //forecast the body move
    GaitInitBody1=GaitInitBody0;
    Design_Walk_Adapt_Fore(&(Gait_Body),&GaitInitBody0,MAX_A_SE2,Gait_T0,&GaitInitBody1);
    struct CJW_JointStruct GaitInitBody2;
    Design_Walk_Adapt_Fore(&(Gait_Body),&GaitInitBody1,MAX_A_SE2,Gait_T0,&GaitInitBody2);
    int TheN=ThisGait->GetGaitN();GaitNum=GaitNum+1;
    ThisGait->GetSwingOrder(GaitType,GaitNum,GaitSwingflag);
    double BodyZInvG[4][4];MathFun.MyGInv((double*)(Gait_Body.G0),(double*)BodyZInvG);
    for(int legi=0;legi<TheN;legi++)
    {
        GaitInitTip1[legi]=Gait_Tip[legi];
        MathFun.MyMatrixCopy(16,(double*)(GaitInitTip0[legi].G),(double*)(GaitInitTip1[legi].G));
#ifndef FOOT_SENSOR
        GaitInitTip0[legi].G[2][3]=0;
        GaitInitTip1[legi].G[2][3]=0;
#endif
        if(GaitSwingflag[legi]==DESIGN_SWI_STATE)
        {
            double TheTempTipG[4][4];MathFun.MyGCompositionG((double *)BodyZInvG,(double*)(Gait_Tip[legi].G),(double*)TheTempTipG);
            double TheTip1G[4][4];MathFun.MyGCompositionG((double *)GaitInitBody1.G0,(double*)TheTempTipG,(double*)TheTip1G);
            double TheTip2G[4][4];MathFun.MyGCompositionG((double *)GaitInitBody2.G0,(double*)TheTempTipG,(double*)TheTip2G);
            GaitInitTip1[legi].G[0][3]=(TheTip1G[0][3]+TheTip2G[0][3])/2;
            GaitInitTip1[legi].G[1][3]=(TheTip1G[1][3]+TheTip2G[1][3])/2; 
            GaitInitTip1[legi].G[2][3]=-WALK_ENTER_SUP_H0; 
        }
    }
    return RIGHT;
}

int CJW_GaitToRobotBased::Design_Walk_Adapt_Based(double time0)
{
    int TheN=ThisGait->GetGaitN();
    //body design for the robot
    ComBody=GaitInitBody0;
    double BodyMaxV[6]={DESIGN_MAX_V0,DESIGN_MAX_V0,DESIGN_MAX_V0,DESIGN_MAX_V0,DESIGN_MAX_V0,DESIGN_MAX_V0};
    double BodyMaxA[6]={DESIGN_MAX_A0,DESIGN_MAX_A0,DESIGN_MAX_A0,DESIGN_MAX_A0,DESIGN_MAX_A0,DESIGN_MAX_A0};
    struct CJW_JointStruct  StartBody=GaitInitBody0;
    /*****************三维运动映射******************************************/
    /*MathFun.MySE3ToSE2((double*)(GaitInitBody0.G0),GaitInitBody0.Body.BodyV,
                       (double*)(StartBody.G0),StartBody.Body.BodyV);
    StartBody.G0[2][3]=GaitInitBody1.G0[2][3];*/ 
    /*****************三维运机身轨迹规划******************************************/          
    double bodyT=Gait_T0;double footT=Gait_T0;
    double body_t=time0;double foot_t=time0;
    if(body_t<=bodyT)
    {
        double tempzero[6]={0};
        ThisGait->Design_SE3(MoveMode,BodyType,StartBody.G0,GaitInitBody1.G0,
                             StartBody.Body.BodyV,GaitInitBody1.Body.BodyV,tempzero,GaitInitBody1.Body.BodyA,
                             BodyMaxV,BodyMaxA,bodyT,body_t,ComBody.G0,ComBody.Body.BodyV,ComBody.Body.BodyA);
    }
    else
    {
        ComBody=GaitInitBody1;
        double thet=body_t-bodyT;
        double BodyDkesi[6]={GaitInitBody1.Body.BodyV[0]*thet,GaitInitBody1.Body.BodyV[1]*thet,GaitInitBody1.Body.BodyV[2]*thet,
                             GaitInitBody1.Body.BodyV[3]*thet,GaitInitBody1.Body.BodyV[4]*thet,GaitInitBody1.Body.BodyV[5]*thet};
        double BodyDG[4][4];MathFun.MyExponent4ToG(BodyDkesi,(double *)BodyDG);
        MathFun.MyGCompositionG((double*)(GaitInitBody1.G0),(double*)BodyDG,(double*)(ComBody.G0));
    }
    //foot trajectory design for the robot
    double norm_time=fmin(footT,foot_t);
    double support_state=BRANCH_SUP_STATE;
    double swing_state=BRANCH_SWI_STATE;
    double exchange_state=BRANCH_BOUNDING_STATE;
    if(norm_time<WALK_ENTER_SUP_TIME)
    {
        support_state=exchange_state+norm_time/WALK_OUT_SWI_TIME*(BRANCH_SUP_STATE-exchange_state);
    }
    // else if(norm_time>(1-WALK_OUT_SUP_COM))
    // {
    //     support_state=BRANCH_BOUNDING_STATE+(1-norm_time)/WALK_OUT_SUP_COM*(BRANCH_SUP_STATE-BRANCH_BOUNDING_STATE);   
    // } 
    if(norm_time>(footT-WALK_OUT_SWI_TIME))
    {
        swing_state=exchange_state+(footT-norm_time)/WALK_OUT_SWI_TIME*(BRANCH_SWI_STATE-exchange_state);   
    }
    // else if(norm_time<WALK_ENTER_SWI_COM)
    // {
    //     swing_state=BRANCH_BOUNDING_STATE+norm_time/WALK_ENTER_SWI_COM*(BRANCH_SWI_STATE-BRANCH_BOUNDING_STATE);
    // }
    for(int legi=0;legi<TheN;legi++)
    {
        if(GaitSwingflag[legi]==DESIGN_SWI_STATE)
        {
            if((foot_t>=(footT/2))&&(RealRobot->GetBranchiState(legi)>=BRANCH_BOUNDING_STATE))
            {
                RealRobot->GetMainModelTipOne(legi,&(ComTip[legi]));
                for(int ii=0;ii<6;ii++){ComTip[legi].V[ii]=0;ComTip[legi].A[ii]=0;}
#ifndef FOOT_SENSOR
                ComTip[legi].G[2][3]=0;
#endif
                GaitInitTip1[legi]=ComTip[legi];
                GaitSwingflag[legi]=DESIGN_SUP_STATE;
                ComBraState[legi]=exchange_state;
            }
            else
            {
                if(foot_t<=footT)
                {
                    ComTip[legi]=GaitInitTip0[legi];
                    double StartP[3]={GaitInitTip0[legi].G[0][3],GaitInitTip0[legi].G[1][3],GaitInitTip0[legi].G[2][3]};
                    double EndP[3]={GaitInitTip1[legi].G[0][3],GaitInitTip1[legi].G[1][3],GaitInitTip1[legi].G[2][3]};
                    double NowP[3];double NowV[3];double NowA[3];
                    ThisGait->Design_swingfoot(SwingType,StartP,EndP,footT,Gait_StepH,foot_t,NowP,NowV,NowA);
                    for(int ii=0;ii<3;ii++)
                    {
                        ComTip[legi].G[ii][3]=NowP[ii];
                        ComTip[legi].V[ii+3]=NowV[ii];
                        ComTip[legi].A[ii+3]=NowA[ii];
                    }
                    ComBraState[legi]=swing_state;
                }
                else//search for the new contact point along Z axis
                {
                    ComTip[legi]=GaitInitTip1[legi];
                    double StartP[3]={GaitInitTip1[legi].G[2][3],0,0};
                    double EndP[3]={GaitInitTip1[legi].G[2][3]-Gait_StepH,0,0};
                    double NowP[3];ThisGait->Design_1DoF(SwingType,StartP,EndP,(footT/2),(foot_t-footT),NowP);
                    ComTip[legi].G[2][3]=NowP[0];
                    ComTip[legi].V[5]=NowP[1];
                    ComTip[legi].A[5]=NowP[2];
#ifndef FOOT_SENSOR
                    ComBraState[legi]=exchange_state;
#endif
                }
            }
        }
        else
        {
            ComTip[legi]=GaitInitTip1[legi];
            ComBraState[legi]=support_state;
        }
        ComBraType[legi]=TIP_DATA_SPE;
    }
    int error=Design_BodyTipToJoint(&ComBody,ComTip,ComBraState,ComBraType);
    return error;
}
/******************adaptive skating gait *****************************************************************************/
int CJW_GaitToRobotBased::Design_Skate_Adapt_Refresh(void)
{
    //选择抬腿序号
    int TheN=ThisGait->GetGaitN();GaitNum=GaitNum+1;
    ThisGait->GetSwingOrder(GaitType,GaitNum,GaitSwingflag);
    int dir=1;if(GaitNum%(TheN/2)){dir=-1;}
    double StepDir=Gait_Margin[3];
    //if(GaitType==QUADRUPED_GAIT_TROT){dir=-dir;}
    //建立浮动基坐标系
    struct CJW_JointStruct StartBody=Gait_Body;
    GaitInitBody1=StartBody;GaitInitBody2=StartBody;
    struct CJW_TipStruct StartTip[BRANCH_N_MAX];
    MathFun.MySE3ToSE2((double*)GaitInitBody0.G0,(double*)StartBody.G0);
    for(int legi=0;legi<TheN;legi++)
    {
        StartTip[legi]=Gait_Tip[legi];
        MathFun.MyGCompositionG((double*)StartBody.G0,(double*)(Gait_Tip[legi].G),(double*)(StartTip[legi].G));
    }
    //浮动基的实际偏置
    double dP_SE2[3]={0};int leg_sup_n=0;
    for(int legi=0;legi<TheN;legi++)
    {
        if(GaitSwingflag[legi]==DESIGN_SUP_STATE)
        {
            for(int ii=0;ii<2;ii++)
            {
               dP_SE2[ii]=dP_SE2[ii]+StartTip[legi].G[ii][3]-GaitInitTip0[legi].G[ii][3];
            }
            leg_sup_n++;
        }
    }
    if(leg_sup_n>0)
    {
        for(int ii=0;ii<2;ii++)
        {
            dP_SE2[ii]=dP_SE2[ii]/leg_sup_n;
        }
    }
    //设计机身运动
    StartBody.G0[0][3]=StartBody.G0[0][3]-dP_SE2[0];
    StartBody.G0[1][3]=StartBody.G0[1][3]-dP_SE2[1];
    double BodyEndDG[4][4]={1,0,0,Gait_Body.G0[0][3],
                            0,cos(dir*Gait_Margin[1]),-sin(dir*Gait_Margin[1]),-dir*Gait_Margin[0]+Gait_Body.G0[1][3],
                            0,sin(dir*Gait_Margin[1]), cos(dir*Gait_Margin[1]),Gait_Body.G0[2][3],
                            0,0,0,1};
    MathFun.MyGCompositionG((double*)(StartBody.G0),(double*)BodyEndDG,(double*)GaitInitBody1.G0);
    MathFun.MyGCompositionG((double*)(StartBody.G0),(double*)Gait_Body.G0,(double*)GaitInitBody2.G0);
    //设计单腿运动
    double R0_SE2[3][3];double P0_SE2[3];
    MathFun.MyGToRP((double*)StartBody.G0,(double*)R0_SE2,(double*)P0_SE2);
    if(GaitType==QUADRUPED_GAIT_PACE)
    {
        Gait_Dir[1]=fabs(Gait_Dir[1]);
    }
    double stepx=-Gait_Dir[1]*sin(StepDir);
    double stepy=dir*Gait_Dir[1]*cos(StepDir);
    for(int legi=0;legi<TheN;legi++)
    {
        StartTip[legi].G[0][3]=StartTip[legi].G[0][3]-dP_SE2[0];
        StartTip[legi].G[1][3]=StartTip[legi].G[1][3]-dP_SE2[1];
        StartTip[legi].G[2][3]=0;
        GaitInitTip0[legi].G[2][3]=0;
        GaitInitTip1[legi]=StartTip[legi];
        GaitInitTip2[legi]=StartTip[legi];
        if(GaitSwingflag[legi]==DESIGN_SWI_STATE)
        {//推地终点的选择
            GaitInitTip1[legi].G[0][3]=GaitInitTip0[legi].G[0][3]+stepx*StartBody.G0[0][0]+stepy*StartBody.G0[0][1];
            GaitInitTip1[legi].G[1][3]=GaitInitTip0[legi].G[1][3]+stepx*StartBody.G0[1][0]+stepy*StartBody.G0[1][1];
        }
    }
    /******被动轮方向*******/
    ExpWheelStructDir=fabs(Gait_Margin[4]);
    if((Gait_Body.Body.BodyV[3]<0)&&(GaitType==QUADRUPED_GAIT_PACE))
    {
        ExpWheelStructDir=-fabs(Gait_Margin[4]);
    }
    return RIGHT;
}
int CJW_GaitToRobotBased::Design_Skate_Adapt_Based(double time)
{
    int error=RIGHT;
    struct CJW_JointStruct  StartBody=GaitInitBody0;
    double duty=Gait_Margin[2];
    if((duty<=0.05)||(duty>=0.95)){duty=0.5;}
    double body_duty=duty;double foot_duty=duty;
    /************设计轮方向***************************/
    double Wheel_dir_dir[BRANCH_N_MAX]={0};
    ThisGait->GetWheelDirection(GaitType,Wheel_dir_dir);
    /*****************三维运动映射******************************************/
    /*MathFun.MySE3ToSE2((double*)(GaitInitBody0.G0),GaitInitBody0.Body.BodyV,
                       (double*)(StartBody.G0),StartBody.Body.BodyV);
    StartBody.G0[2][3]=GaitInitBody1.G0[2][3];*/
    //初始化状态参数
    ComBody=GaitInitBody0;
    int TheN=ThisGait->GetGaitN();
    for(int legi=0;legi<TheN;legi++)
    {
        ComTip[legi]=GaitInitTip0[legi];
        ComBraType[legi]=TIP_DATA_SPE;
        ComBraState[legi] = BRANCH_SUP_STATE;
    }
    //机身运动一个阶段
    double BodyMaxV[6]={DESIGN_MAX_V0,DESIGN_MAX_V0,DESIGN_MAX_V0,DESIGN_MAX_V0,DESIGN_MAX_V0,DESIGN_MAX_V0};
    double BodyMaxA[6]={DESIGN_MAX_A0,DESIGN_MAX_A0,DESIGN_MAX_A0,DESIGN_MAX_A0,DESIGN_MAX_A0,DESIGN_MAX_A0};
    double T1=Gait_T0*body_duty;double T2=Gait_T0-T1;
    if(time<=body_duty*Gait_T0)
    {
        double thet=time;
        double tempzero[6]={0};
        error=error+ThisGait->Design_SE3(MoveMode,BodyType,StartBody.G0,GaitInitBody1.G0,
                                         tempzero,tempzero,tempzero,tempzero,
                                         BodyMaxV,BodyMaxA,T1,thet,ComBody.G0,ComBody.Body.BodyV,ComBody.Body.BodyA);
    }
    else if(time<=Gait_T0)
    {
        double thet=time-T1;
        error=error+ThisGait->Design_SE3_ZERO(MoveMode,BodyType,GaitInitBody1.G0,GaitInitBody2.G0,T2,thet,ComBody.G0,ComBody.Body.BodyV,ComBody.Body.BodyA);
    }
    else
    {
        ComBody=GaitInitBody2;
    }
    //腿分支两个阶段
    double thebody_vx=Gait_Body.Body.BodyV[3];
    double thebody_wz=Gait_Body.Body.BodyV[2];
    double ComBodyR[9];double ComBodyP[3];
    MathFun.MyGToRP((double*)ComBody.G0,ComBodyR,ComBodyP);
    T1=Gait_T0*foot_duty;T2=Gait_T0-T1;
    for (int legi = 0; legi < TheN; legi++)
    {
        double NowP[3] = {0};double NowV[3] = {0};double NowA[3] = {0};
        if (time < 0)
        {
            //Inial
        }
        else if (time <= T1)
        {
            double thet = time;
            double StartP[3] = {GaitInitTip0[legi].G[0][3], GaitInitTip0[legi].G[1][3], GaitInitTip0[legi].G[2][3]};
            double EndP[3] = {GaitInitTip1[legi].G[0][3], GaitInitTip1[legi].G[1][3], GaitInitTip1[legi].G[2][3]};
            error = error + ThisGait->Design_R3_ZERO(SwingType, StartP, EndP, T1, thet, NowP, NowV, NowA);
            for (int ii = 0; ii < 3; ii++)
            {
                ComTip[legi].G[ii][3] = NowP[ii];
                ComTip[legi].V[ii + 3] = NowV[ii];
                ComTip[legi].A[ii + 3] = NowA[ii];
            }
            if (GaitSwingflag[legi] == DESIGN_SWI_STATE)
            {
                double swing_state = BRANCH_SUP_STATE + (BRANCH_BOUNDING_STATE - BRANCH_SUP_STATE) * (thet - SKATING_ENTER_SUP_TIME) / (T1 - SKATING_ENTER_SUP_TIME);
                ComBraState[legi] = fmin(BRANCH_SUP_STATE, swing_state);
            }
            else
            {
                double sup_state = BRANCH_BOUNDING_STATE + (BRANCH_SUP_STATE - BRANCH_BOUNDING_STATE) * thet / SKATING_ENTER_SUP_TIME;
                ComBraState[legi] = fmin(BRANCH_SUP_STATE, sup_state);
            }
        }
        else if (time <= Gait_T0)
        {
            double thet = time - T1;
            if (GaitSwingflag[legi] == DESIGN_SWI_STATE)
            {
                double StartP[3] = {GaitInitTip1[legi].G[0][3], GaitInitTip1[legi].G[1][3], GaitInitTip1[legi].G[2][3]};
                double EndP[3] = {GaitInitTip2[legi].G[0][3], GaitInitTip2[legi].G[1][3], GaitInitTip2[legi].G[2][3]};
                error = error + ThisGait->Design_swingfoot(SwingType, StartP, EndP, T2, Gait_StepH, thet, NowP, NowV, NowA);
                for (int ii = 0; ii < 3; ii++)
                {
                    ComTip[legi].G[ii][3] = NowP[ii];
                    ComTip[legi].V[ii + 3] = NowV[ii];
                    ComTip[legi].A[ii + 3] = NowA[ii];
                }
                double swing_state = fmax(0, BRANCH_BOUNDING_STATE + (T2 - thet) / SKATING_OUT_SWING_TIME * (BRANCH_SWI_STATE - BRANCH_BOUNDING_STATE));
                ComBraState[legi] = swing_state;
            }
            else
            {
                ComTip[legi] = GaitInitTip2[legi];
                ComBraState[legi] = BRANCH_SUP_STATE;
            }
        }
        else
        {
            ComTip[legi] = GaitInitTip2[legi];
            if (GaitSwingflag[legi] == DESIGN_SWI_STATE)
            {
                ComBraState[legi] = BRANCH_BOUNDING_STATE;
            }
            else
            {
                ComBraState[legi] = BRANCH_SUP_STATE;
            }
        }
        /****************设计被动轮方向角*********************/
        /*if (legi < TheN / 2)
        {
            ExpWheelDir[legi] = Wheel_dir_dir[legi] * ExpWheelStructDir + Gait_Dir[2] / 2;
        }
        else
        {
            ExpWheelDir[legi] = Wheel_dir_dir[legi] * ExpWheelStructDir - Gait_Dir[2] / 2;
        }*/
        double G_LegToBody[4][4];MathFun.MyGInvCompositionG((double *)ComBody.G0, (double *)ComTip[legi].G, (double *)G_LegToBody);
        double v_foot[3] = {thebody_vx - G_LegToBody[1][3] * thebody_wz,
                            G_LegToBody[0][3] * thebody_wz, 0};
        ExpWheelDir[legi] = Wheel_dir_dir[legi] * ExpWheelStructDir+SKATING_TURNING_FLAG*atan2(v_foot[1],v_foot[0]); 
    }
    error = error + Design_BodyTipToJoint(&ComBody, ComTip, ComBraState, ComBraType);
    return error;
}

/********************static skating gait**************************************************/
int CJW_GaitToRobotBased::Design_Skate_Static_Refresh(void)
{
    //选择抬腿序号
    int TheN=ThisGait->GetGaitN();
    GaitNum=0;//GaitNum=GaitNum+1;
    for(int legi=0;legi<TheN;legi++)
    {
        GaitSwingflag[legi] = DESIGN_SUP_STATE;
    }
    int swing_id[2]={3,2};
    if (fabs(Gait_Body.Body.BodyV[3]) < fabs(Gait_Body.Body.BodyV[4]))
    {
        return ERROR;
    }
    if (Gait_Body.Body.BodyV[3] >= 0)
    {
        swing_id[0] = 3;
        swing_id[1] = 2;
    }
    else
    {
        swing_id[0] = 0;
        swing_id[1] = 1;
    }
    GaitSwingflag[swing_id[GaitNum%2]] = DESIGN_SWI_STATE;
    int dir=1;if(GaitNum%2){dir=-1;}
    //建立基准坐标系
    struct CJW_JointStruct StartBody=Gait_Body;
    GaitInitBody1=StartBody;GaitInitBody2=StartBody;
    struct CJW_TipStruct StartTip[BRANCH_N_MAX];
    MathFun.MySE3ToSE2((double*)GaitInitBody0.G0,(double*)StartBody.G0);
    for(int legi=0;legi<TheN;legi++)
    {
        StartTip[legi]=Gait_Tip[legi];
        MathFun.MyGCompositionG((double*)StartBody.G0,(double*)(Gait_Tip[legi].G),(double*)(StartTip[legi].G));
    }
    //浮动基的实际偏置
    double dP_SE2[3]={0};int leg_sup_n=0;
    for(int legi=0;legi<TheN;legi++)
    {
        if(GaitSwingflag[legi]==DESIGN_SUP_STATE)
        {
            for(int ii=0;ii<2;ii++)
            {
               dP_SE2[ii]=dP_SE2[ii]+StartTip[legi].G[ii][3]-GaitInitTip0[legi].G[ii][3];
            }
            leg_sup_n++;
        }
    }
    if(leg_sup_n>0)
    {
        for(int ii=0;ii<2;ii++)
        {
            dP_SE2[ii]=dP_SE2[ii]/leg_sup_n;
        }
    }
    //设计机身运动
    StartBody.G0[0][3]=StartBody.G0[0][3]-dP_SE2[0];
    StartBody.G0[1][3]=StartBody.G0[1][3]-dP_SE2[1];
    double safe_angle=0.75*M_PI+swing_id[GaitNum%2]*M_PI/2;
    double BodyEndDG[4][4]={1,0,0,Gait_Margin[0]*cos(safe_angle)+Gait_Body.G0[0][3],
                            0,1,0,Gait_Margin[0]*sin(safe_angle)+Gait_Body.G0[1][3],
                            0,0,1,Gait_Body.G0[2][3],
                            0,0,0,1};
    MathFun.MyGCompositionG((double*)(StartBody.G0),(double*)BodyEndDG,(double*)GaitInitBody1.G0);
    MathFun.MyGCompositionG((double*)(StartBody.G0),(double*)Gait_Body.G0,(double*)GaitInitBody2.G0);
    //设计单腿运动
    double R0_SE2[3][3];double P0_SE2[3];
    MathFun.MyGToRP((double*)StartBody.G0,(double*)R0_SE2,(double*)P0_SE2);
    double StepDir=atan2(Gait_Body.Body.BodyV[4],Gait_Body.Body.BodyV[3]);
    double stepx=-fabs(Gait_Dir[1])*cos(StepDir);
    double stepy=-fabs(Gait_Dir[1])*sin(StepDir);
    for(int legi=0;legi<TheN;legi++)
    {
        StartTip[legi].G[0][3]=StartTip[legi].G[0][3]-dP_SE2[0];
        StartTip[legi].G[1][3]=StartTip[legi].G[1][3]-dP_SE2[1];
        StartTip[legi].G[2][3]=0;
        GaitInitTip0[legi].G[2][3]=0;
        GaitInitTip1[legi]=StartTip[legi];
        GaitInitTip2[legi]=StartTip[legi];
        if(GaitSwingflag[legi]==DESIGN_SWI_STATE)
        {//推地终点的选择
            GaitInitTip1[legi].G[0][3]=GaitInitTip0[legi].G[0][3]+stepx*StartBody.G0[0][0]+stepy*StartBody.G0[0][1];
            GaitInitTip1[legi].G[1][3]=GaitInitTip0[legi].G[1][3]+stepx*StartBody.G0[1][0]+stepy*StartBody.G0[1][1];
        }
    }
    ExpWheelStructDir=StepDir+Gait_Margin[4];
    if(ExpWheelStructDir>M_PI/2)
    {
        ExpWheelStructDir=ExpWheelStructDir-M_PI;
    }
    else if(ExpWheelStructDir< -M_PI/2)
    {
        ExpWheelStructDir=ExpWheelStructDir+M_PI;
    }
    return RIGHT;
}
int CJW_GaitToRobotBased::Design_Skate_Static_Based(double time)
{
    int error=RIGHT;
    if(fabs(Gait_Body.Body.BodyV[3])<fabs(Gait_Body.Body.BodyV[4]))
    {
        return ERROR;
    }
    struct CJW_JointStruct  StartBody=GaitInitBody0;
    double duty=Gait_Margin[2];
    if((duty<=0.05)||(duty>=0.95)){duty=0.5;}
    double body_duty=duty;double foot_duty=duty;
    //初始化状态参数
    ComBody=GaitInitBody0;
    int TheN=ThisGait->GetGaitN();
    for(int legi=0;legi<TheN;legi++)
    {
        ComTip[legi]=GaitInitTip0[legi];
        ComBraType[legi]=TIP_DATA_SPE;
        ComBraState[legi] = BRANCH_SUP_STATE;
    }
    //机身运动一个阶段
    double BodyMaxV[6]={DESIGN_MAX_V0,DESIGN_MAX_V0,DESIGN_MAX_V0,DESIGN_MAX_V0,DESIGN_MAX_V0,DESIGN_MAX_V0};
    double BodyMaxA[6]={DESIGN_MAX_A0,DESIGN_MAX_A0,DESIGN_MAX_A0,DESIGN_MAX_A0,DESIGN_MAX_A0,DESIGN_MAX_A0};
    double T1=Gait_T0*body_duty;double T2=Gait_T0-T1;
    if(time<=T1)
    {
        double thet=time;
        double tempzero[6]={0};
        error=error+ThisGait->Design_SE3(MoveMode,BodyType,StartBody.G0,GaitInitBody1.G0,
                                         tempzero,tempzero,tempzero,tempzero,
                                         BodyMaxV,BodyMaxA,T1,thet,ComBody.G0,ComBody.Body.BodyV,ComBody.Body.BodyA);
    }
    else if(time<=Gait_T0)
    {
        double thet=time-T1;
        error=error+ThisGait->Design_SE3_ZERO(MoveMode,BodyType,GaitInitBody1.G0,GaitInitBody2.G0,T2,thet,ComBody.G0,ComBody.Body.BodyV,ComBody.Body.BodyA);
    }
    else
    {
        ComBody=GaitInitBody2;
    }
    //腿分支两个阶段   
    T1=Gait_T0*foot_duty;T2=Gait_T0-T1;
    double thebody_vx=Gait_Body.Body.BodyV[3];
    double thebody_wz=Gait_Body.Body.BodyV[2];
    for (int legi = 0; legi < TheN; legi++)
    {
        double NowP[3] = {0};double NowV[3] = {0};double NowA[3] = {0};
        if (time < 0)
        {
            //Inial
        }
        else if (time <= T1)
        {
            double thet = time;
            double StartP[3] = {GaitInitTip0[legi].G[0][3], GaitInitTip0[legi].G[1][3], GaitInitTip0[legi].G[2][3]};
            double EndP[3] = {GaitInitTip1[legi].G[0][3], GaitInitTip1[legi].G[1][3], GaitInitTip1[legi].G[2][3]};
            error = error + ThisGait->Design_R3_ZERO(SwingType, StartP, EndP, T1, thet, NowP, NowV, NowA);
            for (int ii = 0; ii < 3; ii++)
            {
                ComTip[legi].G[ii][3] = NowP[ii];
                ComTip[legi].V[ii + 3] = NowV[ii];
                ComTip[legi].A[ii + 3] = NowA[ii];
            }
            if (GaitSwingflag[legi] == DESIGN_SWI_STATE)
            {
                double swing_state = BRANCH_SUP_STATE + (BRANCH_BOUNDING_STATE - BRANCH_SUP_STATE) * (thet - SKATING_ENTER_SUP_TIME) / (T1 - SKATING_ENTER_SUP_TIME);
                ComBraState[legi] = fmin(BRANCH_SUP_STATE, swing_state);
            }
        }
        else if (time <= Gait_T0)
        {
            double thet = time - T1;
            if (GaitSwingflag[legi] == DESIGN_SWI_STATE)
            {
                double StartP[3] = {GaitInitTip1[legi].G[0][3], GaitInitTip1[legi].G[1][3], GaitInitTip1[legi].G[2][3]};
                double EndP[3] = {GaitInitTip2[legi].G[0][3], GaitInitTip2[legi].G[1][3], GaitInitTip2[legi].G[2][3]};
                error = error + ThisGait->Design_swingfoot(SwingType, StartP, EndP, T2, Gait_StepH, thet, NowP, NowV, NowA);
                for (int ii = 0; ii < 3; ii++)
                {
                    ComTip[legi].G[ii][3] = NowP[ii];
                    ComTip[legi].V[ii + 3] = NowV[ii];
                    ComTip[legi].A[ii + 3] = NowA[ii];
                }
                double swing_state = BRANCH_BOUNDING_STATE + (T2 - thet) / SKATING_OUT_SWING_TIME * (BRANCH_SWI_STATE - BRANCH_BOUNDING_STATE);
                ComBraState[legi] = fmax(BRANCH_SWI_STATE, swing_state);
            }
            else
            {
                ComTip[legi] = GaitInitTip2[legi];
            }
        }
        else
        {
            for (int legi = 0; legi < TheN; legi++)
            {
                ComTip[legi] = GaitInitTip2[legi];
                ComBraState[legi] = BRANCH_SUP_STATE;
            }
        }
        /****************设计被动轮方向角*********************/
        if (GaitSwingflag[legi] == DESIGN_SWI_STATE)
        {
            if (ExpWheelStructDir <= 0)
            {
                ExpWheelDir[legi] = ExpWheelStructDir + Gait_Margin[3];
            }
            else
            {
                ExpWheelDir[legi] = ExpWheelStructDir - Gait_Margin[3];
            }
        }
        else
        {
            /*****设置被动轮方向**********************************/
            double G_LegToBody[4][4];
            MathFun.MyGInvCompositionG((double *)ComBody.G0, (double *)ComTip[legi].G, (double *)G_LegToBody);
            double v_foot[3] = {thebody_vx - G_LegToBody[1][3] * thebody_wz,
                                G_LegToBody[0][3] * thebody_wz, 0};
            ExpWheelDir[legi] = atan2(v_foot[1],v_foot[0]);
        }
    }
    error=error+Design_BodyTipToJoint(&ComBody,ComTip,ComBraState,ComBraType);
    return error;
}
int CJW_GaitToRobotBased::Design_Skate_Swizzling_Refresh(void)
{
    int TheN=ThisGait->GetGaitN();
    //建立浮动基坐标系
    GaitInitBody1=Gait_BodySE2;GaitInitBody2=Gait_BodySE2;
    for(int legi=0;legi<TheN;legi++)
    {
       Gait_TipSE2[legi].G[2][3]=0;
    }
    //设计机身运动
    //设计单腿运动
    double Step_Dir[BRANCH_N_MAX]={-1,1,1,-1};
    for(int legi=0;legi<(TheN);legi++)//只设置前腿
    {
        GaitInitTip0[legi].G[2][3]=0;
        GaitInitTip1[legi]=Gait_TipSE2[legi];
        GaitInitTip2[legi]=Gait_TipSE2[legi];
        //推地终点的选择
        GaitInitTip1[legi].G[0][3]=GaitInitTip1[legi].G[0][3]+Step_Dir[legi]*Gait_Dir[1]*Gait_BodySE2.G0[0][1];
        GaitInitTip1[legi].G[1][3]=GaitInitTip1[legi].G[1][3]+Step_Dir[legi]*Gait_Dir[1]*Gait_BodySE2.G0[1][1];  
    }
    /********************************被动轮方向设计*******************/
    if((fabs(Gait_Margin[4])>=(M_PI/18))&&(fabs(Gait_Margin[4])<=(17.0*M_PI/18)))
    {
        ExpWheelStructDir=Gait_Margin[4];
    }
    else
    {
        ExpWheelStructDir=M_PI/4;
    }
    Gait_Body.Body.BodyV[3] = Gait_Dir[1]/(Gait_T0/2)/tan(ExpWheelStructDir);
    Gait_Body.Body.BodyV[2] = Gait_Dir[2]/(Gait_T0/2);
    return RIGHT;
}
int CJW_GaitToRobotBased::Design_Skate_Swizzling_Based(int Startflag, double time)
{
    int error=RIGHT;
    //初始化状态参数
    ComBody=GaitInitBody0;
    int TheN=ThisGait->GetGaitN();
    for(int legi=0;legi<TheN;legi++)
    {
        ComTip[legi]=GaitInitTip0[legi];
        ComBraType[legi]=TIP_DATA_SPE;
        ComBraState[legi] = BRANCH_SUP_STATE;
    }
    //机身运动一个阶段
    double T1=Gait_T0/2;double T2=Gait_T0-T1;
    if(time<=T1)
    {
        double thet=time;
        error=error+ThisGait->Design_SE3_ZERO(MoveMode,BodyType,GaitInitBody0.G0,GaitInitBody1.G0,T1,thet,ComBody.G0,ComBody.Body.BodyV,ComBody.Body.BodyA);
    }
    else if(time<=Gait_T0)
    {
        double thet=time-T1;
        error=error+ThisGait->Design_SE3_ZERO(MoveMode,BodyType,GaitInitBody1.G0,GaitInitBody2.G0,T2,thet,ComBody.G0,ComBody.Body.BodyV,ComBody.Body.BodyA);
    }
    else
    {
        ComBody=GaitInitBody2;
    }
    //double Body_Yaw=atan2(ComBody.G0[1][0],ComBody.G0[0][0]);
    double thebody_vx=Gait_Body.Body.BodyV[3];
    double thebody_wz=Gait_Body.Body.BodyV[2];
    //腿分支两个阶段 
    double Step_Dir[BRANCH_N_MAX]={-1,1,1,-1};
    for (int legi = 0; legi < TheN; legi++)
    {
        double thet = time;
        double StartP[3] = {GaitInitTip0[legi].G[0][3], GaitInitTip0[legi].G[1][3], GaitInitTip0[legi].G[2][3]};
        double EndP[3] = {GaitInitTip1[legi].G[0][3], GaitInitTip1[legi].G[1][3], GaitInitTip1[legi].G[2][3]};
        if (legi >= (TheN / 2))
        {
            if (Startflag > 0) //开始轮滑
            {
                thet = fmax(0, thet - Gait_Margin[2] * Gait_T0);
            }
            else if (Startflag < 0) //结束轮滑
            {
                thet = fmin(Gait_T0,thet - Gait_Margin[2] * Gait_T0 + Gait_T0);
            }
            else //周期运动
            {
                thet = thet - Gait_Margin[2] * Gait_T0;
                if(thet<0)
                {
                    thet =thet + Gait_T0;
                }
                for (int ii = 0; ii < 3; ii++)
                {
                    StartP[ii] = GaitInitTip2[legi].G[ii][3];
                }
            }
        }
        double PushDir=1;
        if (thet > T1)
        {
            PushDir=-1;
            thet = thet - T1;
            for (int ii = 0; ii < 3; ii++)
            {
                StartP[ii] = GaitInitTip1[legi].G[ii][3];
                EndP[ii] = GaitInitTip2[legi].G[ii][3];
            }
        }
        double NowP[3] = {0};double NowV[3] = {0};double NowA[3] = {0};
        error = error + ThisGait->Design_R3_ZERO(DESIGN_CYCLOID, StartP, EndP, T1, thet, NowP, NowV, NowA);
        for (int ii = 0; ii < 3; ii++)
        {
            ComTip[legi].G[ii][3] = NowP[ii];
            ComTip[legi].V[ii + 3] = NowV[ii];
            ComTip[legi].A[ii + 3] = NowA[ii];
        }
        /*****设置被动轮方向**********************************/
        double G_LegToBody[4][4];MathFun.MyGInvCompositionG((double*)ComBody.G0,(double*)ComTip[legi].G,(double*)G_LegToBody);
        double vy_push = Step_Dir[legi]*PushDir*sqrt(NowV[0] * NowV[0] + NowV[1] * NowV[1]);
        double v_foot[3]={thebody_vx-G_LegToBody[1][3]*thebody_wz,
                            G_LegToBody[0][3]*thebody_wz,0};
        if (fabs(v_foot[0]) < 0.01)
        {
            ExpWheelDir[legi] = 0;
        }
        else
        {
            ExpWheelDir[legi] =  atan2((v_foot[1]+vy_push),v_foot[0]);
        }
    }
    error=error+Design_BodyTipToJoint(&ComBody,ComTip,ComBraState,ComBraType);
    return error;
}







