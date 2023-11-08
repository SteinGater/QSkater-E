#include <iostream>

#include <sstream>
#include <fcntl.h>
#include <math.h>

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Joy.h"
#include "robot_msgl/MotorStruct.h"
#include "robot_msgl/TipStruct.h"
#include "robot_msgl/RobotStruct.h"
#include "robot_msgl/ControlStruct.h"
#include "robot_msgl/RobotCommand.h"
#include "robot_msgl/MotorControl.h"
#include "robot_msgl/ComputeControl.h"

using namespace std;



#define ROBOT_AUTO_START       0  //robot auto running
/***********************************************************************************************/
#define M_PI       3.14159265358979323846   // pi
// 定义控制周期/ms
#define CONTROLER_INTERVAL 100
//define error of the control command
#define CONTROL_RIGHT 0
#define CONTROL_WORKING 1
#define CONTROL_WRONG_STATE 2

/////////////////////////#define the body command
//定义手柄按键名称
#define JOY_Y               0
#define JOY_B               1
#define JOY_A               2
#define JOY_X               3
#define JOY_LB              4
#define JOY_RB              5
#define JOY_LT              6
#define JOY_RT              7
#define JOY_BACK            8
#define JOY_START           9
#define JOY_ROCK_A_MID      10
#define JOY_ROCK_B_MID      10
//定义手柄摇杆名称
#define JOY_ROCK_A_LEFRIGHT      0     
#define JOY_ROCK_A_UPDOWN        1
#define JOY_ROCK_B_LEFRIGHT      2
#define JOY_ROCK_B_UPDOWN        3
#define JOY_LEFRIGHT             4
#define JOY_UPDOWN               5
//定义手柄指令的类型
#define JOY_EMERGENCY            -1
#define JOY_NO_FUNCTION          0
#define JOY_BUTTON_FUNCTION      1
#define JOY_ROCK_FUNCTION        2
#define JOY_AUTO_FUNCTION        100

int MainSendFlag=JOY_NO_FUNCTION;//手柄指令类型

/************************************************robot define**************************************************************/
//define gait state
#define CONTROL_EMERGENCY       -1
#define CONTROL_STOP            0
#define CONTROL_STAY            1
#define CONTROL_LEG_MOVE        2
#define CONTROL_WHEEL_MOVE      3
#define CONTROL_LEG_ADAPT       4
#define CONTROL_WHEEL_ADAPT     5
#define CONTROL_LEG_STATIC      6
#define CONTROL_WHEEL_STATIC    7
#define CONTROL_LEGTOWHEEL      10
#define CONTROL_WHEELTOLEG      11
#define CONTROL_CHANGE          12
#define CONTROL_BOUNDING        13
#define CONTROL_TEST            100
//define branch model
#define BRANCH_LEG_MODEL           0
#define BRANCH_WHEEL_MODEL         1
#define BRANCH_ARM_MODEL           2
//define the control model
#define CONTROL_MODEL_POSITION     0
#define CONTROL_MODEL_VELOCITY     1
#define CONTROL_MODEL_TOEQUE       2
//默认指令参数
#define COMMAND_BODY_H0         0.28
#define COMMAND_BODY_LANDH0     0.15
#define COMMAND_FOOT_X0         0.21
#define COMMAND_FOOT_Y0         0.152
#define COMMAND_FOOT_STEPH0     0
#define COMMAND_GAIT_TIME0      1

const double Command_Foot_P0[4][3]={COMMAND_FOOT_X0,-COMMAND_FOOT_Y0,0,
                                    COMMAND_FOOT_X0,COMMAND_FOOT_Y0,0,
                                    -COMMAND_FOOT_X0,COMMAND_FOOT_Y0,0,
                                    -COMMAND_FOOT_X0,-COMMAND_FOOT_Y0,0};
//步态使用的参数 三个字母表示状态下的参数
//W=walk|S=skating;P=position|F=force;T=trot|P=pace|S=Static;
int Skating_Gait_Type=robot_msgl::RobotCommand::Request::GAIT_TROT;

#define ROBOT_EXCHANGE_TIME        4
//walking force trot
#define WFT_TIME             0.25
#define WFT_N0               4
#define WFT_STEPDIR          0.0
#define WFT_STEPL            0.1
#define WFT_STEPA            0.1
#define WFT_STEPH            0.04
#define WFT_FOOTDX           0.03
#define WFT_FOOTDY           0.0
#define WFT_FOOTINX          0.0
#define WFT_FOOTINY          0.0
#define WFT_STEPL_MAX        0.12
#define WFT_TIME_MIN         0.2
//skaing force pace
#define SFP_BODY_H0       0.25
#define SFP_TIME          0.5     //0.6
#define SFP_N0            12    
#define SFP_STEPDIR       0.0   
#define SFP_STEPL         0.02    //0.03
#define SFP_STEPA         0.0   
#define SFP_STEPH         0.03    //0.02
#define SFP_BODYDY        0.06    //0.07
#define SFP_BODYDW        0.1     //0.1
#define SFP_DUTY          0.5     //0.58
#define SFP_PUSHDIR       0.4     //0
#define SFP_W_DIR         (M_PI/9)
#define SFP_FOOTDX        -0.032  //-0.032
#define SFP_FOOTDY        0.001   //0.002
#define SFP_FOOTINX       0.0     //0.0
#define SFP_FOOTINY       -0.007  //-0.007
#define SFP_INC_K         0.00    ///0
//skating force trot
#define SFT_BODY_H0       0.27   //0.27
#define SFT_TIME          0.6    //0.6
#define SFT_N0            12     //21
#define SFT_STEPDIR       0.0
#define SFT_STEPL         0.04   //0.04
#define SFT_STEPA         0.0
#define SFT_STEPH         0.03   //0.03
#define SFT_BODYDY        0.0
#define SFT_BODYDW        0.0
#define SFT_DUTY          0.54   //0.54
#define SFT_PUSHDIR       0.4      //0
#define SFT_W_DIR         (M_PI/9)
#define SFT_FOOTDX        -0.028 //-0.028
#define SFT_FOOTDY        0.0
#define SFT_FOOTINX       0.0
#define SFT_FOOTINY       0.03   //0.03
// skating position static
#define SPS_BODY_H0       0.3   
#define SPS_TIME          2.0    
#define SPS_N0            12     
#define SPS_STEPDIR       0.0
#define SPS_STEPL         0.07   
#define SPS_STEPA         (M_PI/9)
#define SPS_STEPH         0.0   
#define SPS_BODYDY        0.0
#define SPS_BODYDW        0.0
#define SPS_DUTY          0.25
#define SPS_PUSHDIR       0.0
#define SPS_W_DIR         (M_PI/4)     
#define SPS_FOOTDX        -0.028 
#define SPS_FOOTDY        0.0
#define SPS_FOOTINX       0.0
#define SPS_FOOTINY       -0.02
// walking and skating position bounding
#define PB_BODY_HMIN      0.2
#define PB_BODY_HMAX      0.3
#define PB_TJ             0.12
#define PB_THMAX          0.3
#define PB_STEPH          0.05
#define PB_FOOTDXS        0.05
#define PB_FOOTDX         0
#define PB_FOOTDY         0
#define PB_FOOTINX        0
#define PB_FOOTINY        0
//define the send command order
#define COMMAND_NUM_MAX         10         
int Command_i=0;
int Command_Num=0;
int Command_Type[COMMAND_NUM_MAX]={0};
robot_msgl::ComputeControl computemember[COMMAND_NUM_MAX];
robot_msgl::RobotCommand bodymember[COMMAND_NUM_MAX];

//the flag of the maincontrol refresh
robot_msgl::ControlStruct RobotState;
#define COMMAND_ROBOT_INITIAL   0
#define COMMAND_ROBOT_LEG       1
#define COMMAND_ROBOT_WHEEL     2
int Command_Robot_Type=COMMAND_ROBOT_INITIAL;
const int Command_Leg_Type[3][4]={
    BRANCH_WHEEL_MODEL,BRANCH_WHEEL_MODEL,BRANCH_LEG_MODEL,BRANCH_LEG_MODEL,
    BRANCH_LEG_MODEL,BRANCH_LEG_MODEL,BRANCH_LEG_MODEL,BRANCH_LEG_MODEL,
    BRANCH_WHEEL_MODEL,BRANCH_WHEEL_MODEL,BRANCH_WHEEL_MODEL,BRANCH_WHEEL_MODEL};


/**********************************************mini function ******************************************************/
void SetGaitFoot(robot_msgl::RobotCommand *bodysrv,
                double dx,double dy,double inx,double iny)
{
    double AddP[4][3]={dx+inx,dy-iny,0,
                       dx+inx,dy+iny,0,
                       dx-inx,dy+iny,0,
                       dx-inx,dy-iny,0};
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            bodysrv->request.BranchTip[i].P[j] = Command_Foot_P0[i][j]+AddP[i][j];
            bodysrv->request.BranchTip[i].Rw[j] = 0;
        }
        bodysrv->request.BranchModel[i] = RobotState.RealRobot.BranchModel[i];
        bodysrv->request.BranchState[i] = 1;
    }
}
void SetGaitFootS(robot_msgl::RobotCommand *bodysrv,
                double dx,double dy,double inx,double iny)
{
    double AddP[4][3]={dx+inx,dy,0,
                       dx+inx,dy,0,
                       dx-inx,dy+iny,0,
                       dx-inx,dy-iny,0};
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            bodysrv->request.BranchTip[i].P[j] = Command_Foot_P0[i][j]+AddP[i][j];
            bodysrv->request.BranchTip[i].Rw[j] = 0;
        }
        bodysrv->request.BranchModel[i] = RobotState.RealRobot.BranchModel[i];
        bodysrv->request.BranchState[i] = 1;
    }
}
void SetGaitTime(robot_msgl::RobotCommand *bodysrv,double time,double N)
{
    bodysrv->request.time[0]=time;//步态时间
    bodysrv->request.time[1]=N;//步态次数
}
void SetGaitPameter0(robot_msgl::RobotCommand *bodysrv,
                    double dir,double stepL,double stepangle,double stepH)
{
    bodysrv->request.gait_para[0]=dir;//前进方向和X夹角
    bodysrv->request.gait_para[1]=stepL;//步态线步长度
    bodysrv->request.gait_para[2]=stepangle;//步态角步长度
    bodysrv->request.gait_para[3]=stepH;//抬腿高度
}
void SetGaitPameter1(robot_msgl::RobotCommand *bodysrv,
                    double body_dy,double body_wx,double duty,double stepdir,double wheelangle)
{
    bodysrv->request.skating_para[0]=body_dy;//轮滑侧摆距离
    bodysrv->request.skating_para[1]=body_wx;//轮滑侧倾角度
    bodysrv->request.skating_para[2]=duty;//轮滑支撑占空比
    bodysrv->request.skating_para[3]=stepdir;//推地方向
    bodysrv->request.skating_para[4]=wheelangle;//轮滑被动轮偏航角
}
void ResetCommand(robot_msgl::ComputeControl *computesrv,robot_msgl::RobotCommand *bodysrv)
{
    computesrv->request.computeCommand=computesrv->request.COMPUTE_SHUTDOWN;
    bodysrv->request.Robot_Command=CONTROL_EMERGENCY;
    for(int i=0;i<3;i++)
    {
        bodysrv->request.body.P[i]=0;
        bodysrv->request.body.Rw[i]=0;
    }
    bodysrv->request.body.P[2]=COMMAND_BODY_H0;
    SetGaitFoot(bodysrv,0,0,0,0);
    bodysrv->request.gait_type=bodysrv->request.GAIT_TROT;
    SetGaitTime(bodysrv,COMMAND_GAIT_TIME0,1);
    SetGaitPameter0(bodysrv,0,0,0,COMMAND_FOOT_STEPH0);
    SetGaitPameter1(bodysrv,0,0,0.5,0,0);
}
void ResetAllCommand(void)
{
    Command_i=0;
    Command_Num=0;
    for(int ii=0;ii<COMMAND_NUM_MAX;ii++)
    {
        Command_Type[ii]=0;
        ResetCommand(&(computemember[ii]),&(bodymember[ii]));
    }
}
void SetCommandWalkAdapt(robot_msgl::RobotCommand *bodysrv,double dir,double stepLk,double stepak)
{
    bodysrv->request.Robot_Command=CONTROL_LEG_ADAPT;
    bodysrv->request.gait_type=bodysrv->request.GAIT_TROT;
    double thel=stepLk*WFT_STEPL;
    double thet=WFT_TIME;
    if(thel>WFT_STEPL_MAX)//步长和步频优化
    {                                 
        double thev=thel/WFT_TIME;
        thel=WFT_STEPL_MAX;
        thet=fmax(WFT_TIME_MIN,thel/thev);
    }           
    SetGaitFoot(bodysrv,WFT_FOOTDX,WFT_FOOTDY,WFT_FOOTINX,WFT_FOOTINY);
    SetGaitTime(bodysrv,thet,WFT_N0);
    SetGaitPameter0(bodysrv,dir,thel,stepak*WFT_STEPA,WFT_STEPH);
    SetGaitPameter1(bodysrv,0,0,0.5,0,0);
    for(int ii=0;ii<4;ii++)
    {
        bodysrv->request.BranchModel[ii]=BRANCH_LEG_MODEL;
    }
}
void SetCommandSkateAdapt(robot_msgl::RobotCommand *bodysrv,double stepLk, double bodyK)
{
    bodysrv->request.Robot_Command=CONTROL_WHEEL_ADAPT;
    
    if(Skating_Gait_Type==bodysrv->request.GAIT_PACE)
    {
        bodysrv->request.body.P[2]=SFP_BODY_H0;
        SetGaitFoot(bodymember,SFP_FOOTDX,SFP_FOOTDY,SFP_FOOTINX,SFP_FOOTINY);
        bodysrv->request.gait_type=bodysrv->request.GAIT_PACE;
        SetGaitTime(bodysrv,SFP_TIME,SFP_N0);
        SetGaitPameter0(bodysrv,0,stepLk*SFP_STEPL,0,SFP_STEPH);
        SetGaitPameter1(bodysrv,SFP_BODYDY*bodyK,SFP_BODYDW,SFP_DUTY,SFP_PUSHDIR,SFP_W_DIR);
    }
    else
    {
        bodysrv->request.body.P[2]=SFT_BODY_H0;
        SetGaitFootS(bodymember,SFT_FOOTDX,SFT_FOOTDY,SFT_FOOTINX,SFT_FOOTINY);
        bodysrv->request.gait_type=bodysrv->request.GAIT_TROT;
        SetGaitTime(bodysrv,SFT_TIME,SFT_N0);
        SetGaitPameter0(bodysrv,0,stepLk*SFT_STEPL,0,SFT_STEPH);
        SetGaitPameter1(bodysrv,SFT_BODYDY*bodyK,SFT_BODYDW,SFT_DUTY,SFT_PUSHDIR,SFT_W_DIR);
    }
    for(int ii=0;ii<4;ii++)
    {
        bodysrv->request.BranchModel[ii]=BRANCH_WHEEL_MODEL;
    }
}
void SetCommandSkateStatic(robot_msgl::RobotCommand *bodysrv,double stepLk,double stepAK)
{
    bodysrv->request.Robot_Command=CONTROL_WHEEL_STATIC;
    bodysrv->request.gait_type=bodysrv->request.GAIT_PACE;
    bodysrv->request.body.P[2]=SPS_BODY_H0;
    SetGaitFoot(bodymember,SPS_FOOTDX,SPS_FOOTDY,SPS_FOOTINX,SPS_FOOTINY);
    double thet=SPS_TIME;double thek=stepLk;double thea=0;
    if(fabs(stepLk)>1){thet=SPS_TIME/fabs(stepLk);thek=fmin(1.0,fmax(-1.0,stepLk));}
    if(fabs(thek)>0.5){thea=stepAK;}
    SetGaitTime(bodysrv,SPS_TIME,SPS_N0);
    SetGaitPameter0(bodysrv,0,fabs(thek)*SPS_STEPL,thea*SPS_STEPA,SPS_STEPH);
    SetGaitPameter1(bodysrv,SPS_BODYDY,SPS_BODYDW,SPS_DUTY,SPS_PUSHDIR,thek*SPS_W_DIR);
}
void SetCommandBounding(robot_msgl::RobotCommand *bodysrv)
{
    bodysrv->request.Robot_Command=CONTROL_BOUNDING;
    bodysrv->request.body.P[2]=PB_BODY_HMAX;       
    SetGaitFoot(bodysrv,PB_FOOTDX,PB_FOOTDY,PB_FOOTINX,PB_FOOTINY);
    SetGaitTime(bodysrv,PB_TJ,1);
    SetGaitPameter0(bodysrv,0,0,PB_THMAX,PB_STEPH);
    SetGaitPameter1(bodysrv,0,0,0.5,0,0);
}
/**********************************************ros srv function******************************************************/
void JoyReceive(const sensor_msgs::Joy::ConstPtr &MS)
{
    if(MS->buttons[JOY_BACK])//紧急停止
    {
        ResetAllCommand();
        MainSendFlag=JOY_EMERGENCY;
    }
    else if(MS->buttons[JOY_START])//开机标定
    {
        if(RobotState.Robot_State==CONTROL_EMERGENCY)
        {
            ResetAllCommand();
            Command_Num=3;
            computemember[0].request.computeCommand=computemember[0].request.COMPUTE_REALWORD;
            computemember[1].request.computeCommand=computemember[1].request.COMPUTE_IMUOPEN;
            computemember[2].request.computeCommand=computemember[2].request.COMPUTE_RESET_JOINT;
            MainSendFlag=JOY_BUTTON_FUNCTION;
        }
    }
    else if(MS->buttons[JOY_LB])//位置模式
    {
        if (RobotState.Robot_State == CONTROL_STOP)
        {
            ResetAllCommand();
            Command_Num = 1;
            computemember[0].request.computeCommand = computemember[0].request.COMPUTE_JOINT_POSITION;
            MainSendFlag = JOY_BUTTON_FUNCTION;
        }
    }
    else if(MS->buttons[JOY_RB])//力矩模式
    {
        if (RobotState.Robot_State == CONTROL_STOP)
        {
            ResetAllCommand();
            Command_Num = 1;
            computemember[0].request.computeCommand = computemember[0].request.COMPUTE_JOINT_TORQUE;
            MainSendFlag = JOY_BUTTON_FUNCTION;
        }
    }
    else if((MS->buttons[JOY_A])
            ||(MS->buttons[JOY_ROCK_A_MID])||(MS->buttons[JOY_ROCK_B_MID]))
    {//停止运动
        ResetAllCommand();
        Command_Num=1;
        Command_Type[0]=1;
        bodymember[0].request.Robot_Command=CONTROL_STOP;
        MainSendFlag=JOY_BUTTON_FUNCTION;
    }
    else if(MS->buttons[JOY_B])
    {//基本运动复位
        if (RobotState.Robot_State == CONTROL_STOP)
        {
            ResetAllCommand();
            Command_Num = 1;
            Command_Type[0] = 1;
            bodymember[0].request.Robot_Command = CONTROL_STAY;
            MainSendFlag = JOY_BUTTON_FUNCTION;
        }
    }
    else if(MS->buttons[JOY_X])//机器人整体模式切换：初始状态、足行、轮形
    {
        if (RobotState.Robot_State == CONTROL_STOP)
        {
            ResetAllCommand();
            Command_Num = 1;
            Command_Type[0] = 1;
            Command_Robot_Type = (Command_Robot_Type + 1) % 3;
            bodymember[0].request.Robot_Command = CONTROL_CHANGE;
            for (int ii = 0; ii < 4; ii++)
            {
                bodymember[0].request.BranchModel[ii] = Command_Leg_Type[Command_Robot_Type][ii];
            }
            bodymember[0].request.time[0] = ROBOT_EXCHANGE_TIME;
            MainSendFlag = JOY_BUTTON_FUNCTION;
        }
    }
    else if(MS->buttons[JOY_Y])//机身六自由度运动
    {
        if (RobotState.Robot_State == CONTROL_STOP)
        {
            if(MS->buttons[JOY_LT]||MS->buttons[JOY_RT])//跳跃运动
            {
                if (RobotState.Control_Model == CONTROL_MODEL_POSITION)
                {
                    ResetAllCommand();
                    if(MS->buttons[JOY_RT])
                    {
                        Command_Num=1;
                        Command_Type[0] = 1;
                        bodymember[0].request.Robot_Command = CONTROL_STAY;
                        bodymember[0].request.body.P[2] = PB_BODY_HMIN;
                        SetGaitFoot(bodymember, PB_FOOTDXS, PB_FOOTDY, PB_FOOTINX, PB_FOOTINY);
                    }
                    else
                    {
                        Command_Num = 2;
                        Command_Type[0] = 1;
                        bodymember[0].request.Robot_Command = CONTROL_STAY;
                        bodymember[0].request.body.P[2] = PB_BODY_HMIN;
                        SetGaitFoot(bodymember, PB_FOOTDX, PB_FOOTDY, PB_FOOTINX, PB_FOOTINY);
                        Command_Type[1] = 1;
                        SetCommandBounding(&bodymember[1]);
                        //Command_Type[2] = 1;
                        //bodymember[2].request.Robot_Command = CONTROL_STAY;
                    }
                    MainSendFlag = JOY_BUTTON_FUNCTION;
                }
            }
            else//机身六自由度运动
            {
                ResetAllCommand();
                Command_Num = 1;
                Command_Type[0] = 1;
                bodymember[0].request.Robot_Command = CONTROL_TEST;
                MainSendFlag = JOY_BUTTON_FUNCTION;
            }
        }
    }
    // else if(MS->buttons[JOY_LT])
    // {
    //     //MainSendFlag=JOY_BUTTON_FUNCTION;
    // }
    // else if(MS->buttons[JOY_RT])
    // {
    //     //MainSendFlag=JOY_BUTTON_FUNCTION;
    // }
    else if((MS->axes[JOY_LEFRIGHT])||(MS->axes[JOY_UPDOWN]))//上下左右按键
    {
        ResetAllCommand();                         
        Command_Num=1;
        Command_Type[0]=1;
        MainSendFlag=JOY_BUTTON_FUNCTION;
        if(Command_Robot_Type==COMMAND_ROBOT_LEG)//腿式运动
        {
            if(RobotState.Control_Model==CONTROL_MODEL_TOEQUE)
            {
                double thek=1;
                if(RobotState.Robot_State==CONTROL_LEG_ADAPT){thek=2;}
                SetCommandWalkAdapt(bodymember,0,thek*MS->axes[JOY_UPDOWN],MS->axes[JOY_LEFRIGHT]);
            }
            else
            {
                MainSendFlag=JOY_NO_FUNCTION;
            }
        }
        else if(Command_Robot_Type==COMMAND_ROBOT_WHEEL)//轮式运动
        {
            if(MS->axes[JOY_LEFRIGHT]<0)//设置动态步态为trot
            {//trot
                Skating_Gait_Type=robot_msgl::RobotCommand::Request::GAIT_TROT;
            }
            else if(MS->axes[JOY_LEFRIGHT]>0)//设置动态步态为pace
            {//pace
                Skating_Gait_Type=robot_msgl::RobotCommand::Request::GAIT_PACE;
            }
            if(MS->axes[JOY_UPDOWN])
            {
                double thek=1.0;double thekk=1.0;
                if(RobotState.Control_Model==CONTROL_MODEL_TOEQUE)
                {
                    //if(RobotState.Robot_State==CONTROL_WHEEL_ADAPT){thek=2;thekk=1.0+SFP_INC_K;}
                    //SetCommandSkateAdapt(bodymember,thek*fabs(MS->axes[JOY_UPDOWN]),thekk);
                }
                else
                {
                    MainSendFlag=JOY_NO_FUNCTION;
                    //SetCommandSkateStatic(bodymember,thek*MS->axes[JOY_UPDOWN]);
                }
            }
            else
            {
                MainSendFlag=JOY_NO_FUNCTION;
            }
        }
        else 
        {
            bodymember[0].request.Robot_Command=CONTROL_STAY;
            bodymember[0].request.time[0]=2;
            if(MS->axes[JOY_UPDOWN]<0)
            {
                bodymember[0].request.body.P[2]=COMMAND_BODY_LANDH0;
            }
            else
            {
                bodymember[0].request.body.P[2]=COMMAND_BODY_H0;
            }
        }
    }
    else if((MS->axes[JOY_ROCK_A_LEFRIGHT])||(MS->axes[JOY_ROCK_A_UPDOWN])
            ||(MS->axes[JOY_ROCK_B_LEFRIGHT])||(MS->axes[JOY_ROCK_B_UPDOWN])
            ||MainSendFlag==JOY_ROCK_FUNCTION)
    {
        double velocity_k=1.0+fabs(MS->buttons[JOY_LT])+fabs(MS->buttons[JOY_RT]);
        double tempx=velocity_k*(MS->axes[JOY_ROCK_A_UPDOWN]+MS->axes[JOY_ROCK_B_UPDOWN]);
        double tempy=MS->axes[JOY_ROCK_B_LEFRIGHT];
        double tempa=MS->axes[JOY_ROCK_A_LEFRIGHT];
        ResetAllCommand();
        Command_Num=1;
        Command_Type[0]=1;
        MainSendFlag=JOY_ROCK_FUNCTION;
        if(RobotState.Control_Model==CONTROL_MODEL_TOEQUE)
        {
            if(Command_Robot_Type==COMMAND_ROBOT_LEG)
            {
                SetCommandWalkAdapt(bodymember,atan2(tempy,tempx),sqrt(tempx*tempx+tempy*tempy),tempa);
            }
            else if(Command_Robot_Type==COMMAND_ROBOT_WHEEL)
            {
                SetCommandSkateAdapt(bodymember,tempx,(1.0+SFP_INC_K*(velocity_k-1)));
            }
            else
            {
                MainSendFlag=JOY_NO_FUNCTION;
            }
            bodymember[0].request.time[1]=3000; 
        }
        else
        {
            if(Command_Robot_Type==COMMAND_ROBOT_LEG)
            {
                MainSendFlag=JOY_NO_FUNCTION;
            }
            else if(Command_Robot_Type==COMMAND_ROBOT_WHEEL)
            {
                SetCommandSkateStatic(bodymember,tempx,tempa);
            }
            else
            {
                MainSendFlag=JOY_NO_FUNCTION;
            }
        }
    }
    else
    {
        //MainSendFlag=JOY_NO_FUNCTION;
    }
    //printf("the parament is %f + %f + %f \n",bodysrv.request.dir_mode[0]/M_PI*180,bodysrv.request.dir_mode[1],bodysrv.request.dir_mode[2]);
}
/*接受状态器***************************************/
void RecieveMainState(const robot_msgl::ControlStruct::ConstPtr &MS)
{
    RobotState=*MS;
    Command_Robot_Type=RobotState.RealRobot.Robot_Model+1;
}

/*************************************main*************************************************************************/
int main(int argc,char** argv)
{
    //初始化ROS
    ros::init(argc, argv, "robot_joy");
    ros::NodeHandle n;

    //joy
    ros::Subscriber Joy_sub = n.subscribe("joy",1,JoyReceive);
    //接受 main state
    ros::Subscriber MainControlState_sub = n.subscribe("robot_msgl/ControlStruct",1,RecieveMainState);
    //send the body control
    ros::ServiceClient CliBody = n.serviceClient<robot_msgl::RobotCommand> ("robot_msgl/RobotCommand");
    //ros::ServiceClient CliMotor = n.serviceClient<robot_msgl::MotorControl>("robot_msgl/MotorControl");
    ros::ServiceClient CliCompute = n.serviceClient<robot_msgl::ComputeControl>("robot_msgl/ComputeControl");
    //设置控制周期
    ros::Rate loop_rate((double)1000/CONTROLER_INTERVAL);

    //initial the command
    ResetAllCommand();

#ifdef ROBOT_AUTO_START
    //等待主程序启动
    sleep(10);
    computemember[0].request.computeCommand=computemember[0].request.COMPUTE_REALWORD;
    do {sleep(1);} while(!CliCompute.call(computemember[0]));
    computemember[0].request.computeCommand=computemember[0].request.COMPUTE_IMUOPEN;
    do {sleep(1);} while(!CliCompute.call(computemember[0]));
    computemember[0].request.computeCommand=computemember[0].request.COMPUTE_RESET_JOINT;
    do {sleep(1);} while(!CliCompute.call(computemember[0]));
    bodymember[0].request.Robot_Command=CONTROL_STOP;
    do {sleep(1);ros::spinOnce();} while (RobotState.body_error);
    do {sleep(1);}while(!CliBody.call(bodymember[0]));
    ResetAllCommand();
    printf("Joy Initial Robot OK!!!");
#endif
    int joy_once_flag=0;
    // 进入控制循环
    while(ros::ok())
    {
        //-------------控制延时----------------------------------------
        ros::spinOnce();
        loop_rate.sleep();
        //-------------------------------------------------------------
        if(MainSendFlag==JOY_EMERGENCY)//紧急停止
        {
            if(RobotState.Robot_State!=CONTROL_EMERGENCY)
            {
                robot_msgl::RobotCommand stopcommand;
                stopcommand.request.Robot_Command=CONTROL_EMERGENCY;
                CliBody.call(stopcommand);
            }
            else
            {
                MainSendFlag=JOY_NO_FUNCTION;
            }
        }
        else if(MainSendFlag==JOY_BUTTON_FUNCTION)//单次按键状态
        {
            if(Command_i<Command_Num)
            {
                if(joy_once_flag==0)
                {
                    if(Command_Type[Command_i])
                    {
                        if(CliBody.call(bodymember[Command_i]))
                        {
                            int error=bodymember[Command_i].response.error;
                            if(error==0){joy_once_flag=1;}
                        }
                    }
                    else
                    {
                        if(CliCompute.call(computemember[Command_i]))
                        {
                            int error=computemember[Command_i].response.error_codes;
                            if(error==0){joy_once_flag=2;}
                        }
                    }
                }
                else if(joy_once_flag==1)
                {
                    if(RobotState.Robot_State==bodymember[Command_i].request.Robot_Command){joy_once_flag=2;}
                }
                else if((RobotState.Robot_State<=CONTROL_STOP)&&(joy_once_flag==2))
                {
                    Command_i++;
                    joy_once_flag=0;
                }
            }
            else
            {
                MainSendFlag=JOY_NO_FUNCTION;
            }
        }
        else if(MainSendFlag==JOY_ROCK_FUNCTION)//摇杆模式
        {
            if(RobotState.Robot_State!=CONTROL_EMERGENCY)
            {
                CliBody.call(bodymember[0]);
            }
            else
            {
                MainSendFlag=JOY_NO_FUNCTION;
            }
        }
        else //没有按键状态if(MainSendFlag==JOY_NO_FUNCTION)
        {
            ResetAllCommand();
            joy_once_flag=0;
        }
    }
    return 0;
}
