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

using namespace std;

/******************************************************define*****************************************************/
// 定义控制周期/ms
#define CONTROLER_INTERVAL 5
//the define of simulation state
#define REALWORLD   0
#define SIMULATIONWORLD 1

#define BRANCHN  4
#define BODYN    5


//the flag of the 3D simulation refresh
robot_msgl::ControlStruct Robot_State;


/***********************************************************the function of ros serive********************************/
/*接受状态器***************************************/
void RecieveState_3DS(robot_msgl::ControlStruct MS)
{
    //ROS_INFO("accept data");
    Robot_State=MS;
    //printf("the position is %.3f+%.3f+%.3f\n",Robot_State.body.P[0],Robot_State.body.P[1],Robot_State.body.P[2]);
    //cout<<QT_joint_state[3].angle<<endl;
}

/********************************************main***********************************************************************/
int main(int argc,char** argv)
{
    // 初始化ROS
    ros::init(argc, argv, "robot_3dros");
    ros::NodeHandle n;


    // 接受topic
    ros::Subscriber MainControlState_sub_3DS = n.subscribe("/robot_msgl/ControlStruct",1,RecieveState_3DS);

    //send the topic of the rviz
    ros::Publisher rviz_joint_pub=n.advertise<sensor_msgs::JointState>("/joint_states",1);
    sensor_msgs::JointState rviz_joint_state;
    //send gazebo
    //ros::Publisher gazebo_joint_pub=n.advertise<std_msgs::Float64MultiArray>("/robot_gazebo/alljoint_controller/command",1);
    //std_msgs::Float64MultiArray gazebo_joint_state;
    //gazebo_joint_state.data.resize(16);

    tf::TransformBroadcaster rviz_broadcaster;
    geometry_msgs::TransformStamped rviz_odom_trans;
    rviz_odom_trans.header.frame_id="odom";
    rviz_odom_trans.child_frame_id="dummy_link";

    ros::Publisher rviz_odom_pub=n.advertise<nav_msgs::Odometry>("rviz_odom",1);
    nav_msgs::Odometry rviz_odom;
    rviz_odom.header.frame_id="odom";
    rviz_odom.child_frame_id="dummy_link";

    // 设置控制周期
    ros::Rate loop_rate((double)1000/CONTROLER_INTERVAL);
    ros::Time the_ros_time;

    //设置发送的指令
    rviz_joint_state.name.resize(20);
    rviz_joint_state.position.resize(20);
    rviz_joint_state.name[0]="Joint1_1";
    rviz_joint_state.name[1]="Joint1_2";
    rviz_joint_state.name[2]="Joint1_3";
    rviz_joint_state.name[3]="Joint1_foot";
    rviz_joint_state.name[4]="Joint1_wheel";
    rviz_joint_state.name[5]="Joint2_1";
    rviz_joint_state.name[6]="Joint2_2";
    rviz_joint_state.name[7]="Joint2_3";
    rviz_joint_state.name[8]="Joint2_foot";
    rviz_joint_state.name[9]="Joint2_wheel";
    rviz_joint_state.name[10]="Joint3_1";
    rviz_joint_state.name[11]="Joint3_2";
    rviz_joint_state.name[12]="Joint3_3";
    rviz_joint_state.name[13]="Joint3_foot";
    rviz_joint_state.name[14]="Joint3_wheel";
    rviz_joint_state.name[15]="Joint4_1";
    rviz_joint_state.name[16]="Joint4_2";
    rviz_joint_state.name[17]="Joint4_3";
    rviz_joint_state.name[18]="Joint4_foot";
    rviz_joint_state.name[19]="Joint4_wheel";

    // 进入控制循环
    while(ros::ok())
    {
        /********************refresh the 3D rviz view***************************/
        the_ros_time=ros::Time::now();
        //update joint state
        rviz_joint_state.header.stamp=the_ros_time;//the same time

        for(int branchi=0;branchi<BRANCHN;branchi++)
        {
            for(int bodyi=0;bodyi<BODYN;bodyi++)
            {
                int joint_j=branchi*BODYN+bodyi;
                rviz_joint_state.position[joint_j]=Robot_State.RealRobot.MotorState[joint_j].angle;
                //gazebo_joint_state.data[joint_j]=rviz_joint_state.position[joint_j];
            }
        }
        //update the body trans
        rviz_odom_trans.header.stamp=the_ros_time;//the same time
        rviz_odom_trans.transform.translation.x=Robot_State.RealRobot.body.P[0];
        rviz_odom_trans.transform.translation.y=Robot_State.RealRobot.body.P[1];
        rviz_odom_trans.transform.translation.z=Robot_State.RealRobot.body.P[2];
        double odom_temp_o=sqrt(Robot_State.RealRobot.body.Rw[0]*Robot_State.RealRobot.body.Rw[0]+
				Robot_State.RealRobot.body.Rw[1]*Robot_State.RealRobot.body.Rw[1]+
				Robot_State.RealRobot.body.Rw[2]*Robot_State.RealRobot.body.Rw[2]);
        if(odom_temp_o<0.000001)
        {
            rviz_odom_trans.transform.rotation.w=1;
            rviz_odom_trans.transform.rotation.x=0;
            rviz_odom_trans.transform.rotation.y=0;
            rviz_odom_trans.transform.rotation.z=0;
        }
        else
        {
            rviz_odom_trans.transform.rotation.w=cos(odom_temp_o/2.0);
            rviz_odom_trans.transform.rotation.x=Robot_State.RealRobot.body.Rw[0]*sin(odom_temp_o/2.0)/odom_temp_o;
            rviz_odom_trans.transform.rotation.y=Robot_State.RealRobot.body.Rw[1]*sin(odom_temp_o/2.0)/odom_temp_o;
            rviz_odom_trans.transform.rotation.z=Robot_State.RealRobot.body.Rw[2]*sin(odom_temp_o/2.0)/odom_temp_o;
        }
        //update the odom trans
        rviz_odom.header.stamp=the_ros_time;//the same time
        rviz_odom.pose.pose.position.x=rviz_odom_trans.transform.translation.x;
        rviz_odom.pose.pose.position.y=rviz_odom_trans.transform.translation.y;
        rviz_odom.pose.pose.position.z=rviz_odom_trans.transform.translation.z;
        rviz_odom.pose.pose.orientation.x=rviz_odom_trans.transform.rotation.w;
        rviz_odom.pose.pose.orientation.y=rviz_odom_trans.transform.rotation.x;
        rviz_odom.pose.pose.orientation.z=rviz_odom_trans.transform.rotation.y;
        rviz_odom.pose.pose.orientation.w=rviz_odom_trans.transform.rotation.z;
        rviz_odom.twist.twist.linear.x=0;
        rviz_odom.twist.twist.linear.y=0;
        rviz_odom.twist.twist.angular.z=0;

        //if(Robot_State.Simulation_Flag==REALWORLD)
        //{
        //    gazebo_joint_pub.publish(gazebo_joint_state);
        //}
        //else
        //{
            //send the state
            rviz_joint_pub.publish(rviz_joint_state);
            //send the state
            rviz_broadcaster.sendTransform(rviz_odom_trans);
            //send the state
            rviz_odom_pub.publish(rviz_odom);
        //}

        //-------------控制延时----------------------------------------
        ros::spinOnce();
        loop_rate.sleep();
        //-------------------------------------------------------------
    }
    return 0;

}
