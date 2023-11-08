#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <time.h>
#include <stdint.h>
#include <sys/types.h>

#include <pthread.h>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ApplyBodyWrench.h>

#include "UserRobotStructure.h"
#include "Joint_Manage.h"
#include "CJW_FootMeasure.h"

#include "lwa_robot/Eigen_Nonlinear_Equ.h"

using namespace std;

//#define SIMULTION_GAZEBO
//#define TEST_RUNNING
//#define ROBOT_FLOATING
//#define FOOT_SENSOR
/***********************************************************define****************************************************/
//定义控制周期/ms
#define CONTROLER_INTERVAL      5
#define MESSAGE_INTREVAL        10
#define CONTROLER_KERF_N        1
#define CONTROLER_SERVO_N       8
//the define of simulation state
#define REALWORLD               0
#define SIMULATIONWORLD         1
//define error of the control command
#define CONTROL_RIGHT           0
#define CONTROL_WORKING         1
#define CONTROL_WRONG_STATE     2
//define robot main state
#define CONTROL_EMERGENCY       -1 
#define CONTROL_STOP            0
#define CONTROL_STAY            1
#define CONTROL_MOVE            2
#define CONTROL_BOUNDING        3
#define CONTROL_CHANGE          10
#define CONTROL_TEST            100


//define gait stop bounding
#define CONTROL_MOVE_ZERO       0.01

//some special flag
bool Simulation_Flag = SIMULATIONWORLD;
int  Motor_Control_Function = MOTORCOMMAND_POSITION;
int  Control_Model          = CONTROL_MODE_POSITION;
// contol gait
int Control_state = CONTROL_EMERGENCY;
int Control_gait_type = QUADRUPED_GAIT_TROT;
int Control_Command_Flag = 0;
struct robot_msgl::RobotCommand Control_Command;
struct CJW_JointStruct IMU_Data;

/***********************硬件接口*****************************************/
bool IMU_connect_flag = 0;
bool IMU_recieve_flag = 0;
char Foot_Sen_Com_Name[] = "/dev/foot_contact";
bool Foot_connect_flag = 0;
char *Motor_Com_Name[4] = {"/dev/motor_RF", "/dev/motor_LF", "/dev/motor_LH", "/dev/motor_RH"};
bool Motor_connect_flag = 0;
char Servo_Com_Name[] = "/dev/wheel_dir";
bool Servo_connect_flag = 0;

/****************test compute time*************************/
struct timeval Test_start, Test_end, servo_start, servo_end;
unsigned long Test_duration = 0;

/****************多线程数据保护*************************/
bool Motor_Reading_Flag[User_MainBranchN] = {1,1,1,1};
#define MOTOR_READING_PROTECT 5
int Motor_Reading_Protect[User_MainBranchN] = {0,0,0,0};

/**********************************************多线程******************************************/
/* 定义线程四条腿分支的4个并行电机通信线程*/
static void *motor1_pthread(void *arg)
{
    int thebranchn = 0;
    while (1)
    {  
        while (Motor_Reading_Flag[thebranchn]){usleep(50);}
        /* 线程pthread开始运行 */
        int motorerror=Motor_SendRec_Func_OneBranch(thebranchn, epfd[0]);//收发一条分支数据
        if(motorerror==RIGHT)
        {
            Motor_Reading_Protect[thebranchn]=0;
        }
        else//通信中断保护
        {
            Motor_Reading_Protect[thebranchn]++;
            if(Motor_Reading_Protect[thebranchn]>MOTOR_READING_PROTECT)
            {
               Control_state = CONTROL_EMERGENCY; 
            }
        }
        Motor_Reading_Flag[thebranchn] = 1;
        //usleep(1000);
    }
    //std::cout<<"Info: Exit thread 1!!"<<std::endl;
    //pthread_exit(0);
    return NULL;
}
static void *motor2_pthread(void *arg)
{
    int thebranchn = 1;
    while (1)
    {
        while (Motor_Reading_Flag[thebranchn]){usleep(50);}
        /* 线程pthread开始运行 */
        int motorerror=Motor_SendRec_Func_OneBranch(thebranchn, epfd[1]);
        if(motorerror==RIGHT)
        {
            Motor_Reading_Protect[thebranchn]=0;
        }
        else
        {
            Motor_Reading_Protect[thebranchn]++;
            if(Motor_Reading_Protect[thebranchn]>MOTOR_READING_PROTECT)
            {
               Control_state = CONTROL_EMERGENCY; 
            }
        }
        Motor_Reading_Flag[thebranchn] = 1;
    }
    return NULL;
}
static void *motor3_pthread(void *arg)
{
    int thebranchn = 2;
    while (1)
    {
        while (Motor_Reading_Flag[thebranchn]){usleep(50);}
        /* 线程pthread开始运行 */
        int motorerror=Motor_SendRec_Func_OneBranch(thebranchn, epfd[2]);
        if(motorerror==RIGHT)
        {
            Motor_Reading_Protect[thebranchn]=0;
        }
        else
        {
            Motor_Reading_Protect[thebranchn]++;
            if(Motor_Reading_Protect[thebranchn]>MOTOR_READING_PROTECT)
            {
               Control_state = CONTROL_EMERGENCY; 
            }
        }
        Motor_Reading_Flag[thebranchn] = 1;
    }
    
    return NULL;
}
static void *motor4_pthread(void *arg)
{
    int thebranchn = 3;
    while (1)
    {
        while (Motor_Reading_Flag[thebranchn]){usleep(50);}
        /* 线程pthread开始运行 */
        int motorerror=Motor_SendRec_Func_OneBranch(thebranchn, epfd[3]);
        if(motorerror==RIGHT)
        {
            Motor_Reading_Protect[thebranchn]=0;
        }
        else
        {
            Motor_Reading_Protect[thebranchn]++;
            if(Motor_Reading_Protect[thebranchn]>MOTOR_READING_PROTECT)
            {
               Control_state = CONTROL_EMERGENCY; 
            }
        }
        Motor_Reading_Flag[thebranchn] = 1;
    }
    return NULL;
}
/*****************************************ros srv function*************************************************/
//////////////////////////////////////////open and shut down the compute
bool service_ComputeControl(robot_msgl::ComputeControl::Request &req,
                            robot_msgl::ComputeControl::Response &res)
{
    if (req.computeCommand == req.COMPUTE_SHUTDOWN)
    {
    }
    else if (req.computeCommand == req.COMPUTE_SIMULATE)
    {
        Simulation_Flag = SIMULATIONWORLD;
    }
    else if (req.computeCommand == req.COMPUTE_REALWORD)
    {
        Simulation_Flag = REALWORLD;
    }
    else if (req.computeCommand == req.COMPUTE_IMUOPEN)
    {
        IMU_connect_flag = 1;
    }
    else if (req.computeCommand == req.COMPUTE_IMUCLOSE)
    {
        IMU_connect_flag = 0;
    }
    else if (req.computeCommand == req.COMPUTE_JOINT_POSITION)
    {
        User_Gait.Design_Robot_Emergency();
        Motor_Control_Function = MOTORCOMMAND_POSITION;
        Control_Model          = CONTROL_MODE_POSITION;
    }
    else if (req.computeCommand == req.COMPUTE_JOINT_VELOCITY)
    {
        Motor_Control_Function = MOTORCOMMAND_VELOCITY;
        Control_Model          = CONTROL_MODE_VELOCITY;
    }
    else if (req.computeCommand == req.COMPUTE_JOINT_TORQUE)
    {
        Motor_Control_Function = MOTORCOMMAND_TORQUE;
        Control_Model          = CONTROL_MODE_FORCE;
    }
    else if ((req.computeCommand == req.COMPUTE_RESET_JOINT)&&(Control_state == CONTROL_EMERGENCY))
    {
#ifndef SIMULTION_GAZEBO
        Motor_Joint_Initial_Calibration();
        User_RobotReal.MainResetPVKef(1.0*CONTROLER_INTERVAL/1000/CONTROLER_KERF_N,Robot_PV_Esti_Para);
#endif
    }
    //printf("function is %d\n",User_MotorFunction[0][0]);
    res.error_codes = CONTROL_RIGHT;
    ROS_INFO("compute service request: %d state: %d", req.computeCommand,res.error_codes);
    return true;
}

/********************************************* 舵机控制 服务例程******************************************/
bool service_MotorControl(robot_msgl::MotorControl::Request &req,
                          robot_msgl::MotorControl::Response &res)
{
    ROS_INFO("servo service request:\ncmd:%d,servoID:%d,value:%f", req.Command, req.ID, req.value);
    int min1 = req.ID / MOTOR_BRANCHN_N;
    int max1 = min1 + 1;
    int min2 = req.ID % MOTOR_BRANCHN_N;
    int max2 = min2 + 1;
    if (req.ID == 0)
    {
        min1 = 0;
        max1 = User_MainBranchN;
        min2 = 0;
        max2 = MOTOR_BRANCHN_N;
    }
    for (int branchi = min1; branchi < max1; branchi++)
    {
        for (int bodyi = min2; bodyi < max2; bodyi++)
        {
            UserMotorExp[branchi][bodyi].ID = User_MotorIDMap[branchi][bodyi];
            if (req.Command == MOTORCOMMAND_POSITION)
            {
                UserMotorExp[branchi][bodyi].angle = req.value;
            }
            else if (req.Command == MOTORCOMMAND_VELOCITY)
            {
                UserMotorExp[branchi][bodyi].velocity = req.value;
            }
            else if (req.Command == MOTORCOMMAND_TORQUE)
            {
                UserMotorExp[branchi][bodyi].torque = req.value;
            }
            else if (req.Command == MOTORCOMMAND_ENABLE)
            {
                UserMotorExp[branchi][bodyi].Enable = req.value;
            }
            else if (req.Command == MOTORCOMMAND_ACTION)
            {
                User_MotorFunction[branchi][bodyi] = req.value;
            }
            else if (req.Command == MOTORCOMMAND_CLEAR)
            {
            }
            else
            {
                res.error = false;
                return false;
            }
        }
    }
    return true;
}
/********************************************* 机身整体控制 服务例程******************************************/
bool service_RobotCommand(robot_msgl::RobotCommand::Request &req,
                          robot_msgl::RobotCommand::Response &res)
{
    ROS_INFO("Robot service:Command:%d", req.Robot_Command);
    if (req.Robot_Command == CONTROL_EMERGENCY)
    {
        //User_Gait.SetInitState();
        //User_Gait.Design_Robot_Emergency();
        //MapBranchiJointToMotorJoint(&User_RobotExp);
        //失能电机
        Control_Command.request.Robot_Command = CONTROL_EMERGENCY;
        Control_state = CONTROL_EMERGENCY;
        res.error = CONTROL_RIGHT;
    }
    else
    {
        Control_Command_Flag = 0;
        if (req.Robot_Command == CONTROL_STOP)
        {
            Control_Command_Flag = 1;
        }
        else if ((req.Robot_Command == CONTROL_STAY) || (req.Robot_Command == CONTROL_TEST) || (req.Robot_Command == CONTROL_BOUNDING))
        {
            if (Control_state == CONTROL_STOP)
            {
                int flagbranch = 0;
                for (int branchi = 0; branchi < User_MainBranchN; branchi++)
                {
                    flagbranch = flagbranch + (req.BranchModel[branchi] == User_RobotReal.GetBranchiModel(branchi));
                }
                Control_Command_Flag = (flagbranch >= User_MainBranchN);
            }
        }
        else if (req.Robot_Command == CONTROL_CHANGE)
        {
            if (Control_state == CONTROL_STOP)
            {
                Control_Command_Flag = 1;
            }
        }
        else if(req.Robot_Command  == CONTROL_MOVE)
        {
            Control_Command_Flag = 1;
        }
        if (Control_Command_Flag)
        {
            Control_Command.request.Robot_Command = req.Robot_Command;
            Control_Command.request.body = req.body;
            for (int branchi = 0; branchi < User_MainBranchN; branchi++)
            {
                Control_Command.request.BranchModel[branchi] = req.BranchModel[branchi];
                Control_Command.request.BranchState[branchi] = req.BranchState[branchi];
                Control_Command.request.BranchTip[branchi] = req.BranchTip[branchi];
            }
            Control_Command.request.gait_type=req.gait_type;
            Control_Command.request.time[0] = req.time[0];
            Control_Command.request.time[1] = req.time[1];
            Control_Command.request.gait_para[0] = req.gait_para[0];
            Control_Command.request.gait_para[1] = req.gait_para[1];
            Control_Command.request.gait_para[2] = req.gait_para[2];
            Control_Command.request.gait_para[3] = req.gait_para[3];
            Control_Command.request.skating_para[0] = req.skating_para[0];
            Control_Command.request.skating_para[1] = req.skating_para[1];
            Control_Command.request.skating_para[2] = req.skating_para[2];
            Control_Command.request.skating_para[3] = req.skating_para[3];
            Control_Command.request.skating_para[4] = req.skating_para[4];
        }
        else
        {
            res.error = CONTROL_WRONG_STATE;
        }
    }

    ROS_INFO("Body service request:%d", res.error);
    return true;
}
/**********************************************IMU接受数据******************************************/
void imuCallback(const sensor_msgs::Imu::ConstPtr &imu)
{
    if (IMU_connect_flag)
    {
        // std::cout<<"INFO: in imu"<<std::endl;
        double qq[4]={imu->orientation.x,imu->orientation.y,imu->orientation.z,imu->orientation.w};
        double tempRw[3];User_Math.MyQuaternionToExponent3(qq,tempRw);
        double theR[3][3];User_Math.MyExponent3ToR(tempRw, (double *)theR);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                IMU_Data.G0[i][j] = theR[i][j];
            }
        }
        IMU_Data.Body.BodyV[0] = (imu->angular_velocity.x);//+IMU_Data.Body.BodyV[0])/2;
        IMU_Data.Body.BodyV[1] = (imu->angular_velocity.y);//+IMU_Data.Body.BodyV[1])/2;
        IMU_Data.Body.BodyV[2] = (imu->angular_velocity.z);//+IMU_Data.Body.BodyV[2])/2;
        IMU_Data.Body.BodyA[3] = imu->linear_acceleration.x-theR[2][0]*WORLD_GRAVITY;
        IMU_Data.Body.BodyA[4] = imu->linear_acceleration.y-theR[2][1]*WORLD_GRAVITY;
        IMU_Data.Body.BodyA[5] = imu->linear_acceleration.z-theR[2][2]*WORLD_GRAVITY;
    }
    IMU_recieve_flag=1;
}
/******************************************************仿真gazebo的状态参数************************************************/
void RecieveJointState_Gazebo(const sensor_msgs::JointState::ConstPtr &MS)
{
    for (int branchi = 0; branchi < User_MainBranchN; branchi++)
    {
        //motor joint of leg
        for (int bodyi = 0; bodyi < MOTOR_BRANCHN_N; bodyi++)
        {
            int thek = branchi * User_MainBranchBodyN + bodyi;
            UserMotorReal[branchi][bodyi].angle = MS->position[thek];
            UserMotorReal[branchi][bodyi].velocity = MS->velocity[thek];
            UserMotorReal[branchi][bodyi].torque = MS->effort[thek];
        }
        //direction of wheels
        UserServoReal[branchi].angle = MS->position[branchi * User_MainBranchBodyN +3];
        UserServoReal[branchi].velocity = MS->velocity[branchi * User_MainBranchBodyN +3];
        UserServoReal[branchi].torque = MS->effort[branchi * User_MainBranchBodyN +3];
        //wheel state
        //UserWheelReal[branchi].angle = MS->position[branchi * User_MainBranchBodyN +4];
        UserWheelReal[branchi].velocity = MS->velocity[branchi * User_MainBranchBodyN +4];
        UserWheelReal[branchi].torque = MS->effort[branchi * User_MainBranchBodyN +4];
    
    }
    //printf("the msec is %d\n",MS.header.stamp.sec*1000+MS.header.stamp.nsec/1000000);
}

/*********************************设置机器人控制参数*************************************/
//w=walk;s=skating//t=trot;p=pace;//a=adapt
//对角自适应足行步态的机身PID和最大加速度
double User_BodyFKp_wta[6] = {800, 800, 300, 10, 10, 120}; 
double User_BodyFKd_wta[6]={20,20,10,50,50,18};     
double User_Gait_Walk_MaxAcc[6]={10,10,1,0.4,0.3,10};
//对角自适应轮滑步态的机身PID
double User_BodyFKp_sta[6] = {800, 800, 300, 90, 90, 120}; 
double User_BodyFKd_sta[6]={20,20,10,20,20,18};
//同侧自适应轮滑步态的机身PID
double User_BodyFKp_spa[6] = {800.0, 800.0, 400.0, 90.0,  90.0, 120.0}; 
double User_BodyFKd_spa[6]={40.0,  40.0,  20.0,  20.0,  40.0, 18.0};
void MyRobot_SetControlParameter(int type)
{
    double tempzero[6] = {0};
    double *theFKp = User_BodyFKp0;
    double *theFKd = User_BodyFKd0;
    double *theInsideKp = tempzero;
    double *theInsideKd = tempzero;
    if (type == CONTROL_MOVE)
    {
        if (User_RobotReal.GetRobotModel() == ROBOT_MODEL_LEG)
        {
            theFKp = User_BodyFKp_wta;
            theFKd = User_BodyFKd_wta;
        }
        else if (User_RobotReal.GetRobotModel() == ROBOT_MODEL_WHEEL)
        {
            if (Control_gait_type == QUADRUPED_GAIT_PACE)
            {
                theFKp = User_BodyFKp_spa;
                theFKd = User_BodyFKd_spa;
            }
            else
            {
                theFKp = User_BodyFKp_sta;
                theFKd = User_BodyFKd_sta;
            }
        }
    }
    if (User_RobotReal.GetRobotModel() != ROBOT_MODEL_LEG)
    {
        theInsideKp = User_InsideKp;
        theInsideKd = User_InsideKd;
    }
    User_Gait.SetControlParameter(theFKp, theFKd, User_BodyVKp,
                                  User_Qua_S, User_Qua_W,
                                  User_BodyAccMax, User_BodyAccMin,
                                  theInsideKp, theInsideKd,
                                  User_SwingKp, User_SwingKd);
}
void MyRobot_GetInitialParameter(void)
{
    XmlRpc::XmlRpcValue param_list;
    //电机参数
    if(ros::param::get("/robot_main/motor_pos_K",param_list))
    {Motor_Joint_Set_PID(MOTORCOMMAND_POSITION,param_list[0],param_list[1]);}
    if(ros::param::get("/robot_main/motor_vel_K",param_list))
    {Motor_Joint_Set_PID(MOTORCOMMAND_VELOCITY,param_list[0],param_list[1]);}
    if(ros::param::get("/robot_main/motor_tor_K",param_list))
    {Motor_Joint_Set_PID(MOTORCOMMAND_TORQUE,param_list[0],param_list[1]);}
    //最优二次参数
    if(ros::param::get("/robot_main/quadprog_S",param_list))
    {for(int ii=0;ii<6;ii++){User_Qua_S[ii]=param_list[ii];}}
    if(ros::param::get("/robot_main/quadprog_W",param_list))
    {for(int ii=0;ii<3;ii++){User_Qua_W[ii]=param_list[ii];}}
    //摆动腿力控制参数
    if(ros::param::get("/robot_main/swing_Kp",param_list))
    {for(int ii=0;ii<3;ii++){User_SwingKp[ii]=param_list[ii];}}
    if(ros::param::get("/robot_main/swing_Kd",param_list))
    {for(int ii=0;ii<3;ii++){User_SwingKd[ii]=param_list[ii];}}
    //机身力控制参数
    if(ros::param::get("/robot_main/body_Kp",param_list))
    {for(int ii=0;ii<6;ii++){User_BodyFKp0[ii]=param_list[ii];}}
    if(ros::param::get("/robot_main/body_Kd",param_list))
    {for(int ii=0;ii<6;ii++){User_BodyFKd0[ii]=param_list[ii];}}
    //轮滑内力参数
    if(ros::param::get("/robot_main/inside_Kp",param_list))
    {for(int ii=0;ii<3;ii++){User_InsideKp[ii]=param_list[ii];}}
    if(ros::param::get("/robot_main/inside_Kd",param_list))
    {for(int ii=0;ii<3;ii++){User_InsideKd[ii]=param_list[ii];}}
    //卡尔曼滤波参数
    if(ros::param::get("/robot_main/body_PV_estimation",param_list))
    {for(int ii=0;ii<6;ii++){Robot_PV_Esti_Para[ii]=param_list[ii];}}
    //对角足行力控制参数
    if(ros::param::get("/robot_main/walk_trot_adapt/body_Kp",param_list))
    {for(int ii=0;ii<6;ii++){User_BodyFKp_wta[ii]=param_list[ii];}}
    if(ros::param::get("/robot_main/walk_trot_adapt/body_Kd",param_list))
    {for(int ii=0;ii<6;ii++){User_BodyFKd_wta[ii]=param_list[ii];}}
    if(ros::param::get("/robot_main/walk_trot_adapt/acc_max",param_list))
    {for(int ii=0;ii<6;ii++){User_Gait_Walk_MaxAcc[ii]=param_list[ii];}}
    //对角轮滑力控制参数
    if(ros::param::get("/robot_main/skating_trot_adapt/body_Kp",param_list))
    {for(int ii=0;ii<6;ii++){User_BodyFKp_sta[ii]=param_list[ii];}}
    if(ros::param::get("/robot_main/skating_trot_adapt/body_Kd",param_list))
    {for(int ii=0;ii<6;ii++){User_BodyFKd_sta[ii]=param_list[ii];}}
    //同侧轮滑力控制参数
    if(ros::param::get("/robot_main/skating_pace_adapt/body_Kp",param_list))
    {for(int ii=0;ii<6;ii++){User_BodyFKp_spa[ii]=param_list[ii];}}
    if(ros::param::get("/robot_main/skating_pace_adapt/body_Kd",param_list))
    {for(int ii=0;ii<6;ii++){User_BodyFKd_spa[ii]=param_list[ii];}}
}
/******************************************************main*************************************************************/
int main(int argc, char **argv)
{
    //初始化ROS 加载初始参数
    ros::init(argc, argv, "robot_main");
    ros::NodeHandle n;
    MyRobot_GetInitialParameter();

#ifdef SIMULTION_GAZEBO
    //gazebo model state
    ros::Subscriber ALLJoints_sub = n.subscribe("/robot_gazebo/joint_states", 1, RecieveJointState_Gazebo); //joint special
    //ros::Subscriber LegBumper1_sub = n.subscribe("/robot_gazebo/Leg1_bumper_states",1,RecieveBumper1State_Gazebo);
    //ros::Subscriber LegBumper2_sub = n.subscribe("/robot_gazebo/Leg2_bumper_states",1,RecieveBumper2State_Gazebo);
    //ros::Subscriber LegBumper3_sub = n.subscribe("/robot_gazebo/Leg3_bumper_states",1,RecieveBumper3State_Gazebo);
    //ros::Subscriber LegBumper4_sub = n.subscribe("/robot_gazebo/Leg4_bumper_states",1,RecieveBumper4State_Gazebo);
    //ros::Publisher ALLJoints_pub = n.advertise<std_msgs::Float64MultiArray>("/robot_gazebo/alljoint_controller/command", 1);
    //std_msgs::Float64MultiArray AllJointCommand;
    //AllJointCommand.data.resize(16);
    //AllJointCommand.data[0]=0.0;
    //ALLJoints_pub.publish(AllJointCommand);
    Simulation_Flag = REALWORLD;
    IMU_connect_flag=1;
#else
    //开启电机通信
    if (Inital_Motor_Connect(Motor_Com_Name) == RIGHT)
    {
        Motor_connect_flag = 1;
        //初始motor 多线程实现并行通信
        pthread_t motor1_pth;
        pthread_t motor2_pth;
        pthread_t motor3_pth;
        pthread_t motor4_pth;

        if ((pthread_create(&motor1_pth, NULL, motor1_pthread, NULL)) == -1)
        {
            printf("create error!");
            return 1;
        }
        else
        {
            std::cout << "Info: Thread 1 created successfully!" << std::endl;
        }
        if ((pthread_create(&motor2_pth, NULL, motor2_pthread, NULL)) == -1)
        {
            printf("create error!");
            return 1;
        }
        else
        {
            std::cout << "Info: Thread 2 created successfully!" << std::endl;
        }
        if ((pthread_create(&motor3_pth, NULL, motor3_pthread, NULL)) == -1)
        {
            printf("create error!");
            return 1;
        }
        else
        {
            std::cout << "Info: Thread 3 created successfully!" << std::endl;
        }
        if ((pthread_create(&motor4_pth, NULL, motor4_pthread, NULL)) == -1)
        {
            printf("create error!");
            return 1;
        }
        else
        {
            std::cout << "Info: Thread 4 created successfully!" << std::endl;
        }
    }
    //开启轮的方向通信
    if(Dynamixel_Init(Servo_Com_Name,B1000000)==RIGHT)
    {
        Servo_connect_flag=1;
    }
#ifdef FOOT_SENSOR
    //开启足地力通信
    if (Foot_Init(Foot_Sen_Com_Name, B230400) == RIGHT)
    {
        Foot_connect_flag = 1;
    }
#endif

#endif


    fflush(stdout);

    // 设置控制周期
    ros::Rate loop_rate((double)1000 / CONTROLER_INTERVAL);

    //开启机器人服务
    ros::ServiceServer srvMotor = n.advertiseService("robot_msgl/MotorControl", service_MotorControl);
    ros::ServiceServer srvControl = n.advertiseService("robot_msgl/RobotCommand", service_RobotCommand);
    ros::ServiceServer srvCompute = n.advertiseService("robot_msgl/ComputeControl", service_ComputeControl);

    //发布机器人状态
    ros::Publisher MainControlState_pub = n.advertise<robot_msgl::ControlStruct>("robot_msgl/ControlStruct", 10);
    robot_msgl::ControlStruct Public_State;

    //订阅IMU的消息
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu/data", 10, imuCallback);

    //初始化理论机器人的状态
    MyRobot_Init();
    double KerTime=1.0*CONTROLER_INTERVAL/1000/CONTROLER_KERF_N;
    User_RobotReal.MainResetPVKef(KerTime,Robot_PV_Esti_Para);
    Set_Joint_ALL_Enable(0);//Set_Motor_ALL_Enable(0);

    sleep(1);
    /****** 定义主函数变量**********************/
    struct CJW_JointStruct Control_Body;
    struct CJW_TipStruct Control_Tip[User_MainBranchN];
    int Control_BranchModel[User_MainBranchN];
    double Control_BranchState[User_MainBranchN];
    double Control_T0 = 0;
    int Control_period_N = 1;
    double Control_N_cycle = 1;
    double Control_dir_mode[3] = {0};
    double Control_StepH = 0;
    double Control_Margin[5] = {0};
    /****** define i**********************/
    int control_once_flag = 0;
    int control_num = 0;
    int control_main_i = 0;
    int control_design_error = RIGHT;
    int control_joint_error =RIGHT;
    int control_body_error =RIGHT;

    int control_servo_i = 0;
    //int control_test_i=0;
#ifdef TEST_RUNNING
    gettimeofday(&Test_start, NULL);
#endif

    /************************************************* 进入控制循环****************************/
    while (ros::ok())
    {
        /**********************************************************************************************/
        if (Control_state == CONTROL_EMERGENCY)
        {
            control_once_flag = 0;
            control_num = 0;
            control_main_i = 0;
            control_design_error = 0;
            User_Gait.SetInitState();
            User_Gait.Design_Robot_Emergency();
            Set_Joint_ALL_Enable(0);// Set_Motor_ALL_Enable(0);
            Motor_Control_Function = MOTORCOMMAND_POSITION;
            Control_Model=CONTROL_MODE_POSITION;
            if ((Control_Command.request.Robot_Command == CONTROL_STOP) && (Control_Command_Flag))
            {//使能电机
                double body_temp[3]={0};
                User_RobotReal.GetMainBodyP(body_temp);
                User_RobotExp.SetMainBodyP(body_temp);
                Set_Joint_ALL_Enable(1);//Set_Motor_ALL_Enable(1);
                Control_state = Control_Command.request.Robot_Command;
            }
        }
        else
        {
            if (control_main_i == 0)//new command flag
            {
                if (Control_Command_Flag)
                {
                    if (Control_Command.request.Robot_Command != CONTROL_STOP)
                    {
                        Control_state = Control_Command.request.Robot_Command;
                        //aim initial state
                        MapTipMsgToJointStruct(&(Control_Command.request.body), &Control_Body);
                        for (int branchi = 0; branchi < User_MainBranchN; branchi++)
                        {
                            Control_BranchModel[branchi] = Control_Command.request.BranchModel[branchi];
                            Control_BranchState[branchi] = Control_Command.request.BranchState[branchi];
                            MapTipMsgToTipStruct(&(Control_Command.request.BranchTip[branchi]), &(Control_Tip[branchi]));
                        }
                        Control_gait_type=Control_Command.request.gait_type;
                        Control_period_N = (int)(1.0 * Control_Command.request.time[0] * 1000 / CONTROLER_INTERVAL);
                        Control_T0 = 1.0 * Control_period_N * CONTROLER_INTERVAL / 1000;
                        Control_N_cycle = control_num + Control_Command.request.time[1];
                        Control_dir_mode[0] = Control_Command.request.gait_para[0];//运动方向（与X方向夹角）
                        Control_dir_mode[1] = Control_Command.request.gait_para[1];//步长
                        Control_dir_mode[2] = Control_Command.request.gait_para[2];//步角
                        Control_Body.Body.BodyV[0] = 0;
                        Control_Body.Body.BodyV[1] = 0;
                        Control_Body.Body.BodyV[2] = Control_dir_mode[2]/Control_T0;
                        Control_Body.Body.BodyV[3] = Control_dir_mode[1] * cos(Control_dir_mode[0])/Control_T0;
                        Control_Body.Body.BodyV[4] = Control_dir_mode[1] * sin(Control_dir_mode[0])/Control_T0;
                        Control_Body.Body.BodyV[5] = 0;
                        Control_StepH = Control_Command.request.gait_para[3];//抬腿高度
                        Control_Margin[0] = Control_Command.request.skating_para[0];//轮滑侧向摆动距离
                        Control_Margin[1] = Control_Command.request.skating_para[1];//轮滑翻滚角度
                        Control_Margin[2] = Control_Command.request.skating_para[2];//轮滑推地占空比//静态轮滑相位差
                        Control_Margin[3] = Control_Command.request.skating_para[3];//轮滑推地方向
                        Control_Margin[4] = Control_Command.request.skating_para[4];//轮滑被动轮偏航角
                        //User_Gait_Qua.SetBranchModel(Control_BranchModel);
                        //for different command
                        User_Gait.SetGaitMotionType(Control_gait_type, DESIGN_MOVE_DECOUPE, DESIGN_CUBIC, DESIGN_CYCLOID);
                        User_Gait.SetGaitParameter(&(Control_Body), Control_Tip,Control_T0,Control_StepH,Control_dir_mode,Control_Margin);
                        User_Gait.SetInitState();
                    }
                    else
                    {
                        Control_N_cycle = control_num + 1;
                    }
                    Control_Command_Flag = 0;
                } 
            }
            if (Control_state > CONTROL_STOP)
            {
                double control_tmp_time = 1.0 * control_main_i * CONTROLER_INTERVAL / 1000;
                //the designed gait
                if (Control_state == CONTROL_STAY)
                {
                    control_design_error = User_Gait.Design_MainTip_PPmode(Control_BranchState, control_tmp_time);
                    Control_N_cycle = 0;
                    if (control_main_i >= Control_period_N)
                    {
                        control_once_flag = 1;
                    }
                }
                else if (Control_state == CONTROL_TEST)
                {
                    double theAmp0[6]={-0.2,-0.2,-0.2,-0.05,-0.05,-0.05};
                    int theN0=11;int theN=1.0*control_main_i/Control_period_N;
                    int thenn=1.0*theN/2;int thedir=1-(theN%2)*2;
                    double theAmp[6]={0};theAmp[thenn]=1.0*thedir*theAmp0[thenn];
                    double thetime=control_tmp_time-Control_T0*theN;
                    control_design_error = User_Gait.Design_MainTip_P0Test(Control_BranchState, theAmp, thetime);
                    Control_N_cycle = 0;
                    if (control_main_i >=theN0*Control_period_N)
                    {
                        control_once_flag = 1;
                    }
                }
                else if (Control_state == CONTROL_MOVE)//move for robot
                {
                    if (control_main_i == 0)
                    {
                        User_Gait.SetInitState();
                        User_Gait.Design_Move_Refresh(User_Gait_Walk_MaxAcc);
                    }
                    if(Control_period_N<2)
                    {
                        Control_period_N=2;
                    }
                    int StartFlag=0;
                    if(control_num==0)
                    {
                        StartFlag=1;
                    }
                    else if(control_num==(Control_N_cycle-1))
                    {
                        StartFlag=-1;
                    }
                    control_design_error = User_Gait.Design_Move_Run(StartFlag,control_tmp_time);
                    if (control_main_i >= Control_period_N)
                    {
                        control_once_flag = 1;
                    }
                }
                else if (Control_state == CONTROL_BOUNDING)
                {
                    int theflag=0;
                    control_design_error = User_Gait.Design_Bounding_Kinetic(control_tmp_time,&theflag);
                    Control_N_cycle = 0;
                    if (theflag)
                    {
                        control_once_flag = 1;
                    }
                }
                else if (Control_state==CONTROL_CHANGE) //transform between wheel and leg
                {
                    double control_tmp_time = 1.0 * control_main_i * CONTROLER_INTERVAL / 1000;
                    control_design_error = User_Gait.Design_BetweenLegAndWheel_Kinetic(&(User_MainBody0), User_MainTip0,Control_BranchModel, control_tmp_time);
                    Control_N_cycle = 0;
                    if (control_main_i >= Control_period_N)
                    {
                        control_once_flag = 1;
                    }
                }
                //time is running
                control_main_i++;//printf("time i is %d \n",control_main_i);
                if (control_once_flag)
                {
                    control_num++;
                    if (control_num >= (Control_N_cycle - 1))
                    {
                        Control_dir_mode[0] = 0;
                        Control_dir_mode[1] = 0;
                        Control_dir_mode[2] = 0;
                        Control_Body.Body.BodyV[2] = 0;
                        Control_Body.Body.BodyV[3] = 0;
                        Control_Body.Body.BodyV[4] = 0;
                        Control_Body.Body.BodyA[2] = 0;
                        Control_Body.Body.BodyA[3] = 0;
                        Control_Body.Body.BodyA[4] = 0;
                    }
                    if (control_num >= Control_N_cycle)
                    {
                        double TempBodyV[6];
                        User_RobotExp.GetMainBodyV(TempBodyV);
                        double VelocityBodyAbs = fabs(TempBodyV[3]) + fabs(TempBodyV[4]) + 0 * fabs(TempBodyV[5]);
                        if ((VelocityBodyAbs < CONTROL_MOVE_ZERO) || (Control_N_cycle == 0))
                        {
                            Control_state = CONTROL_STOP;
                            Control_Command.request.Robot_Command = Control_state;
                            //User_Gait.SetInitState();
                            //control_design_error = User_Gait.Design_Robot_Stop();
                        }
                    }
                    control_main_i = 0;
                    control_once_flag = 0;
                }
            }
            else //Control_state=CONTROL_STOP;
            {
                control_once_flag = 0;
                control_num = 0;
                control_main_i = 0;
                control_design_error = User_Gait.Design_Robot_Stop();
            }
            /*************************control for the robot**************************/
            MyRobot_SetControlParameter(Control_state);
            control_design_error = control_design_error + User_Gait.Control_Robot_Based(Control_Model);
        }
        /****************************多线程结束数据****************************************/
#ifndef SIMULTION_GAZEBO
        if (Motor_connect_flag && (Simulation_Flag == REALWORLD))
        {
            while ((Motor_Reading_Flag[0] + Motor_Reading_Flag[1] + Motor_Reading_Flag[2] + Motor_Reading_Flag[3]) < 4)
            {
                usleep(100);
            }
        }
#endif
        /************************************************************************************/
        //send the data to the robot
        if((control_design_error == 0)&&(User_RobotExp.CheckALLLimit()==0))
        {
            MapBranchiJointToMotorJoint(&User_RobotExp,Motor_Control_Function);
        }
        else
        {
            ROS_INFO("time %.3f has error:%s(code: %d).",(1.0*control_main_i*CONTROLER_INTERVAL/1000),Gait_Error_Name[control_design_error%6].c_str(),control_design_error);
        }
        /***********************************************************************************************************/
        if (Simulation_Flag == REALWORLD)
        {
            control_servo_i=(control_servo_i+1)%CONTROLER_SERVO_N;
            if((Servo_connect_flag)&&(control_servo_i==0))
            {
                for(int servo_i=0;servo_i<User_MainBranchN;servo_i++)
                {
                    DY_SetTorqueState(servo_i+1,UserServoExp[servo_i].Enable);
                    DY_SetPositionValue(servo_i+1,UserServoExp[servo_i].angle);
                    //DY_FindPosition(servo_i+1);
                }
            }
        }
        /***********************************控制延时************************************************************************/
#ifdef TEST_RUNNING
        gettimeofday(&Test_end, NULL);
        Test_duration = (CLOCKS_PER_SEC * (Test_end.tv_sec - Test_start.tv_sec) + (Test_end.tv_usec - Test_start.tv_usec));//*0.5+Test_duration*0.5;
        if (Test_duration >= CONTROLER_INTERVAL*1000)
        {
            printf("using out the time is %d\n", Test_duration);
        }
        bool TimeOk = loop_rate.sleep(); //if(TimeOk==false){printf("using ros the time is %d\n",Test_duration);}
        ros::spinOnce();
        gettimeofday(&Test_start, NULL);
#else
        bool TimeOk = loop_rate.sleep(); 
        ros::spinOnce();
#endif
        /********************************关节角度数值***************************************************************/
        if(Simulation_Flag==SIMULATIONWORLD)//simulation model do not need real robot
        {//without feel and the expected robot is the real robot
            for (int branchi = 0; branchi < User_MainBranchN; branchi++)
            {
                for (int bodyi = 0; bodyi < MOTOR_BRANCHN_N; bodyi++)
                {
                    double thisaccelation=1.0*(UserMotorExp[branchi][bodyi].velocity-UserMotorReal[branchi][bodyi].velocity)/CONTROLER_INTERVAL*1000.0;
                    UserMotorReal[branchi][bodyi] = UserMotorExp[branchi][bodyi];
                    UserMotorReal[branchi][bodyi].accelation=thisaccelation;
                }
                if(Servo_connect_flag==0){UserServoReal[branchi]=UserServoExp[branchi];}
                UserWheelReal[branchi]=UserWheelExp[branchi];
                //User_RobotReal.SetBranchiState(branchi, User_RobotExp.GetBranchiState(branchi));
            }
        }
        MapMotorJointToBranchiJoint(&User_RobotReal);
        /****************************多线程及其他开始读取数据****************************************/
#ifndef SIMULTION_GAZEBO
        if(Simulation_Flag == REALWORLD)
        {
            if(Motor_connect_flag)
            {
               for (int branchi = 0; branchi < User_MainBranchN; branchi++)
                {
                    Motor_Reading_Flag[branchi] = 0;
                } 
            }
            if(Servo_connect_flag)//转向读取
            {
                //Dynamixel_ReadData();
            }
            if(Foot_connect_flag)//足地反馈监测
            {
                double Foot_F0[8]={0};
                if(Foot_Analysis(Foot_F0)==RIGHT){}
            }
        }
        /**************************the real robot compute the state***************************/
        control_joint_error=User_RobotReal.CheckALLLimit();
        if(control_joint_error){Control_state = CONTROL_EMERGENCY;Set_Joint_ALL_Enable(0);}//safe
#endif
        for (int branchi = 0; branchi < User_MainBranchN; branchi++)
        {                                      
            double angle0[BRANCH_BODY_MAX]={0};
            User_RobotReal.GetBranchiModelAngleP(branchi,angle0);
            if(angle0[2]<0)//当前构型分类
            {
                User_RobotReal.SetBranchiModel(branchi,BRANCH_WHEEL_MODEL);
            }
            else
            {
                User_RobotReal.SetBranchiModel(branchi,BRANCH_LEG_MODEL);
            }
            User_RobotReal.SetBranchiState(branchi,User_RobotExp.GetBranchiState(branchi));
        }
        //计算关节能够计算的相对状态
        User_RobotReal.RobotAngleToAllStateBody();
        if (IMU_connect_flag==0)
        {
            User_RobotExp.GetMainBodyG(IMU_Data.G0);
            User_RobotReal.SetMainBodyG(IMU_Data.G0);
            User_RobotExp.GetMainBodyV(IMU_Data.Body.BodyV);
            User_RobotReal.SetMainBodyV(IMU_Data.Body.BodyV);
            User_RobotExp.GetMainBodyA(IMU_Data.Body.BodyA);
            User_RobotReal.SetMainBodyA(IMU_Data.Body.BodyA);
        }
#ifdef ROBOT_FLOATING
        User_RobotReal.MainAnglePTransToTipG();
        User_RobotReal.MainAngleVTransToTipV();
        User_RobotReal.MainAngleFTransToTipF();
#else
        for(int thei=0;thei<CONTROLER_KERF_N;thei++)
        {
            User_RobotReal.MainTipPVTransToBodyPV(&IMU_Data,KerTime);
        }
        control_body_error=MyRobot_Check_Body(&User_RobotReal);
        if(control_body_error){Control_state = CONTROL_EMERGENCY;Set_Joint_ALL_Enable(0);}//safe
#endif  
        MapRobotToPublicState(&User_RobotReal, &(Public_State.RealRobot));
        MapRobotToPublicState(&User_RobotExp, &(Public_State.ExpRobot));
        //publice robot state
        Public_State.Robot_State = Control_state;
        Public_State.Control_Model = Motor_Control_Function;
        Public_State.Simulation_Flag = Simulation_Flag;
        Public_State.Gait_Type=Control_gait_type;
        Public_State.design_error=control_design_error;
        Public_State.body_error=control_body_error;
        Public_State.leg_error_num=control_joint_error;
        Public_State.header.stamp = ros::Time::now();
        MainControlState_pub.publish(Public_State);
    } // end-while
    /********************************close motion**************************************/
#ifndef SIMULTION_GAZEBO
    if (Motor_connect_flag)
    {
        MOTOR_send motor_ss;
        MOTOR_recv motor_rr;
        motor_ss.id = 0xBB; //motor ID
        motor_ss.mode = 0;  //switch to servo mode

        modify_data(&motor_ss);
        broadcast(Motor_fdi[0], &motor_ss);
        broadcast(Motor_fdi[1], &motor_ss);
        std::cout << "Info: Stop motor!!!" << std::endl;
        extract_data(&motor_rr);
    }
#endif
    std::cout << "Info: Stop program!!!" << std::endl;

    return 0;
}
