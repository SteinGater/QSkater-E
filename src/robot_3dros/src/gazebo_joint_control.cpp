#include <iostream>
#include <sstream>
#include <fcntl.h>

#include "ros/ros.h"
#include "ros/time.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include "robot_msgl/MotorStruct.h"
#include "robot_msgl/TipStruct.h"
#include "robot_msgl/RobotStruct.h"
#include "robot_msgl/ControlStruct.h"
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ApplyBodyWrench.h>

using namespace std;

/******************************************************define*****************************************************/
// 定义控制周期/ms
#define CONTROLER_INTERVAL 1

#define BRANCHN  4
#define BODYN    5
#define JOINTN   20

/***********************关节运动指令*************************/
#define MOTORCOMMAND_POSITION       0
#define MOTORCOMMAND_VELOCITY       1
#define MOTORCOMMAND_TORQUE         2
#define MOTORCOMMAND_ENABLE         3
#define MOTORCOMMAND_ACTION         4
#define MOTORCOMMAND_CLEAR          5
//the flag of the 3D simulation refresh
robot_msgl::ControlStruct Robot_State;
robot_msgl::MotorStruct Robot_Joint[JOINTN];


/***********************************************************the function of ros serive********************************/
/*接受状态器***************************************/
void RecieveState_3DS(const robot_msgl::ControlStruct::ConstPtr &MS)
{
    //ROS_INFO("accept data");
    Robot_State=*MS;
}
/******************************************************仿真gazebo的状态参数************************************************/
void RecieveJointState_Gazebo(const sensor_msgs::JointState::ConstPtr &MS)
{
    for(int n=0;n<JOINTN;n++)
    {
        Robot_Joint[n].angle=MS->position[n];
        Robot_Joint[n].velocity=MS->velocity[n];
        Robot_Joint[n].torque=MS->effort[n];
    }
}
/*******************************电机控制参数***************************************************/
double Motor_Control_P[JOINTN]={0};
double Motor_Control_I[JOINTN]={0};
double Motor_Max_Torque=0;
void GetInitialParameter(void)
{
    XmlRpc::XmlRpcValue param_list;
    //printf("reading data \n");
    if(ros::param::get("/robot_gazebo/Joint_controller/Max_T",param_list)){Motor_Max_Torque=param_list;}
    for(int ii=0;ii<BRANCHN;ii++)
    {
        string name0=to_string(ii+1);
        int jj=ii*BODYN;
        //电机参数
        if(ros::param::get("/robot_gazebo/Joint_controller/Joint"+name0+"_1/pid/p",param_list)){Motor_Control_P[jj+0]=param_list;}
        if(ros::param::get("/robot_gazebo/Joint_controller/Joint"+name0+"_2/pid/p",param_list)){Motor_Control_P[jj+1]=param_list;}
        if(ros::param::get("/robot_gazebo/Joint_controller/Joint"+name0+"_3/pid/p",param_list)){Motor_Control_P[jj+2]=param_list;}
        if(ros::param::get("/robot_gazebo/Direction_controller/Joint"+name0+"_foot/pid/p",param_list)){Motor_Control_P[jj+3]=param_list;}
        if(ros::param::get("/robot_gazebo/Wheel_controller/Joint"+name0+"_wheel/pid/p",param_list)){Motor_Control_P[jj+4]=param_list;}
        if(ros::param::get("/robot_gazebo/Joint_controller/Joint"+name0+"_1/pid/i",param_list)){Motor_Control_I[jj+0]=param_list;}
        if(ros::param::get("/robot_gazebo/Joint_controller/Joint"+name0+"_2/pid/i",param_list)){Motor_Control_I[jj+1]=param_list;}
        if(ros::param::get("/robot_gazebo/Joint_controller/Joint"+name0+"_3/pid/i",param_list)){Motor_Control_I[jj+2]=param_list;}
        if(ros::param::get("/robot_gazebo/Direction_controller/Joint"+name0+"_foot/pid/i",param_list)){Motor_Control_I[jj+3]=param_list;}
        if(ros::param::get("/robot_gazebo/Wheel_controller/Joint"+name0+"_wheel/pid/i",param_list)){Motor_Control_I[jj+4]=param_list;}
    }
}
/********************************************main***********************************************************************/
int main(int argc, char **argv)
{
    // 初始化ROS
    ros::init(argc, argv, "robot_joint_control");
    ros::NodeHandle n;

    GetInitialParameter();

    // 接受topic
    ros::Subscriber MainControlState_sub_3DS = n.subscribe("/robot_msgl/ControlStruct", 1, RecieveState_3DS);
    ros::Subscriber ALLJoints_sub = n.subscribe("/robot_gazebo/joint_states", 1, RecieveJointState_Gazebo); //joint special
    //ros::Subscriber ALLModels_sub = n.subscribe("/gazebo/model_states", 1, RecieveModelState_Gazebo);
    //send gazebo
    ros::Publisher Joints_pub = n.advertise<std_msgs::Float64MultiArray>("/robot_gazebo/Joint_controller/command", 1);
    ros::Publisher Dirs_pub = n.advertise<std_msgs::Float64MultiArray>("/robot_gazebo/Direction_controller/command", 1);
    ros::Publisher Wheels_pub = n.advertise<std_msgs::Float64MultiArray>("/robot_gazebo/Wheel_controller/command", 1);
    std_msgs::Float64MultiArray JointCommand;
    std_msgs::Float64MultiArray DirCommand;
    std_msgs::Float64MultiArray WheelCommand;
    JointCommand.data.resize(12);
    DirCommand.data.resize(BRANCHN);
    WheelCommand.data.resize(BRANCHN);
    // 设置控制周期
    ros::Rate loop_rate((double)1000 / CONTROLER_INTERVAL);

    // 进入控制循环
    while (ros::ok())
    {
        if (Robot_State.Robot_State < 0)
        {
            for (int ii = 0; ii < 12; ii++)
            {
                JointCommand.data[ii] = 0;
            }
            for (int ii = 0; ii < BRANCHN; ii++)
            {
                DirCommand.data[ii] = 0;
                WheelCommand.data[ii] = 0;
            }
        }
        else
        {
            for (int branchi = 0; branchi < BRANCHN; branchi++)
            {
                //the PD joint controller
                for (int bodyi = 0; bodyi < BODYN; bodyi++)
                {
                    int thej = branchi * BODYN + bodyi;
                    double motor_kp = Motor_Control_P[thej];
                    double motor_kv = Motor_Control_I[thej];
                    double JointForce = motor_kp * (Robot_State.ExpRobot.MotorState[thej].angle - Robot_Joint[thej].angle) + motor_kv * (Robot_State.ExpRobot.MotorState[thej].velocity - Robot_Joint[thej].velocity) + Robot_State.ExpRobot.MotorState[thej].torque;
                    if (JointForce > Motor_Max_Torque)
                    {
                        JointForce = Motor_Max_Torque;
                    }
                    else if (JointForce < -Motor_Max_Torque)
                    {
                        JointForce = -Motor_Max_Torque;
                    }
                    if (bodyi < 3) //the leg joint
                    {
                        JointCommand.data[branchi * 3 + bodyi] = JointForce;
                    }
                    else if (bodyi == 3)//the dir servo
                    {
                        DirCommand.data[branchi] = JointForce;
                    }
                    else if (bodyi == 4)//the wheel
                    {
                        WheelCommand.data[branchi] = JointForce;
                    }
                }
            }
        }
        Joints_pub.publish(JointCommand);
        Dirs_pub.publish(DirCommand);
        Wheels_pub.publish(WheelCommand);
        //-------------控制延时----------------------------------------
        ros::spinOnce();
        loop_rate.sleep();
        //-------------------------------------------------------------
    }
    return 0;
}
