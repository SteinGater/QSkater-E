#ifndef QTMAIN_H
#define QTMAIN_H

#include <math.h>
#include <QMainWindow>
#include <QDebug>
#include <QLabel>
#include <QComboBox>
#include <QStringList>
#include <QObject>
#include <QDialog>
#include <QTextEdit>
#include <QPushButton>
#include <QDockWidget>
#include <QtCore>
#include <QMessageBox>


#include "messagethread.h"
#include "std_msgs/Int32.h"
#include "ros/ros.h"
#include "robot_msgl/MotorStruct.h"
#include "robot_msgl/TipStruct.h"
#include "robot_msgl/RobotStruct.h"
#include "robot_msgl/ControlStruct.h"
#include "robot_msgl/RobotCommand.h"
#include "robot_msgl/MotorControl.h"
#include "robot_msgl/ComputeControl.h"

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
//#include <rviz/config.h">
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <rviz/default_plugin/view_controllers/orbit_view_controller.h>
#include <rviz/view_manager.h>
#include <rviz/yaml_config_reader.h>

using namespace std;

#define M_PI       3.14159265358979323846   // pi
#define FLOATPRE    3
#define SHOWZERO    0.0001

// 定义控制周期/ms
#define CONTROLER_INTERVAL      200
#define CONNECT_TIME            5

#define BRANCHN                 4
#define BRANCH_BODYN            5
#define BRANCH_MODELN           3
#define BRANCH_STATEN           2

//////////////////////////////////robot state
//define branch state
#define BRANCH_LEG_MODEL           0
#define BRANCH_WHEEL_MODEL         1
#define BRANCH_ARM_MODEL           2

//define the Leg state
#define LEG_SUP_STATE   1
#define LEG_SWI_STATE   0
//define the WHEEL state
#define WHEEL_SUP_STATE   1
#define WHEEL_SWI_STATE   0
//define the Arm state
#define ARM_MOVE_A   0
#define ARM_MOVE_B   1
const QString BRANCH_MODEL_NAME[BRANCH_MODELN]={QString::fromUtf8("腿"),QString::fromUtf8("轮"),QString::fromUtf8("臂")};
const QString BRANCH_STATE_NAME[BRANCH_MODELN][BRANCH_STATEN]={QString::fromUtf8("摆动"),QString::fromUtf8("支撑"),
                                                                QString::fromUtf8("摆动"),QString::fromUtf8("支撑"),
                                                                 "A","B"};

//define gait type
const QString CONTROL_MODEL[3]={QString::fromUtf8("位置"),QString::fromUtf8("速度"),QString::fromUtf8("力矩")};
const QString GAIT_TYPE[3]={"trot","pace","bound"};

//事实错误代码
const QString Gait_Design_Error[6]={"OK",QString::fromUtf8("机身模式"),
                                        QString::fromUtf8("腿分支模式"),
                                        QString::fromUtf8("位置逆解"),
                                        QString::fromUtf8("速度逆解"),
                                        QString::fromUtf8("足地力分配")};
const QString Robot_Body_Error[6]={"OK",QString::fromUtf8("水平姿态限制"),
                                        QString::fromUtf8("位置X限制"),
                                        QString::fromUtf8("位置Y限制"),
                                        QString::fromUtf8("位置Z上限"),
                                        QString::fromUtf8("位置Z下限")};

//define the initial Poisition
#define User_MainBody_X 	0.21
#define User_MainBody_Y 	0.152
#define User_MainBody_Z 	0
//the body command
#define User_MainBody_ComX  0.030
#define User_MainBody_ComY  0.007
//the define main state
#define MAIN_STOP           -1
//the define of simulation state
#define REALWORLD           0
#define SIMULATIONWORLD     1

//the joint command
#define MOTORCOMMAND_POSITION       0
#define MOTORCOMMAND_VELOCITY       1
#define MOTORCOMMAND_TORQUE         2
#define MOTORCOMMAND_ENABLE         3
#define MOTORCOMMAND_ACTION         4
#define MOTORCOMMAND_CLEAR          5


namespace Ui {
class QTMain;
}

class QTMain : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit QTMain(QWidget *parent = 0);
    ~QTMain();

private:
    Ui::QTMain *ui;


protected:
    void timerEvent(QTimerEvent *event);//refresh the view timer
private:
    int qtreviewtime;

private:
    //the qtcontrol state
    QLabel *p_state_bar;
    QComboBox BranchiModelBox[BRANCHN];

    int the_mode_connect;//链接模式选择
    int the_connect_flag;//链接状态标志
    int the_connect_time;

    //the flag of the maincontrol refresh
    robot_msgl::ControlStruct RobotState;
    double SumForce;
    int SumAble;
    int Sumerror;

    //ROS message recieve part
private:
    ros::NodeHandle* pNode;
    messageThread *pMythread;

    //send the task the robot
    robot_msgl::MotorControl motorcontrolsrv;
    robot_msgl::RobotCommand robotcommadsrv;
    robot_msgl::ComputeControl computecontrolsrv;

    //ros::ServiceClient servoclient;
    ros::ServiceClient motorclient;
    ros::ServiceClient bodyclient;
    ros::ServiceClient computeclient;

    QString MySetNum(double data);
    void SetFootPosition(double dx,double dy);
    void SetGaitParameter(double dir,double stepl,double stepa,double stepH,
                            double maginY,double rorateX,double duty,double stepdir,double wheelangle);

private Q_SLOTS:
    //receive the state of msgs
    void handleMainControlStateChange(robot_msgl::ControlStructConstPtr ms);

    //the ui function
private:
    int the_open_3dros_flag;
    rviz::RenderPanel *render_panel;
    rviz::VisualizationManager* rviz_manager;
    rviz::Config rviz_config;

private slots:
    //the function of menu bar
    void on_actionManualpan_triggered();
    void on_actionAutopan_triggered();
    void on_actionJointtest_triggered();
    void on_actionGazebo_triggered();
    void send_the_joint_base(int ID,int Command,double value);
    void on_actionClearAllError_triggered();
    void on_actionTORQUE_ON_triggered();
    void on_actionTORQUE_OFF_triggered();
    void send_the_compute_base(int Command);
    void on_actionSIMULATE_triggered();
    void on_actionREALWORD_triggered();
    void on_actionOPENIMU_triggered();
    void on_actionCLOSEIMU_triggered();
    void on_actionPosition_triggered();
    void on_actionVelocity_triggered();
    void on_actionTorque_triggered();
    void on_actionReset_triggered();

    //the function of manual
    void Mani_Read_Data(void);
    void Mani_Send_Command(void);
    void SetParameter(int command,int gait_type);
    void on_m_maincommand_currentIndexChanged(int index);
    void on_m_gait_type_currentIndexChanged(int index);
    void on_actionStandup_triggered();
    void on_actionLaydown_triggered();
    void on_m_send_command_clicked();
    void on_m_send_stop_clicked();

    //the function of auto
    void send_easymove_base(double easy_dir[3]);
    void on_a_easyfront_clicked();
    void on_a_easystop_clicked();
    void on_a_easyback_clicked();
    void on_a_easyleft_clicked();
    void on_a_easyright_clicked();
    void on_a_easylrorate_clicked();
    void on_a_easyrrorate_clicked();
    void on_a_easybodyshow_clicked();
    void on_pushButton_rviz_show_clicked();
    void on_dockWidget_rviz_visibilityChanged(bool visible);
};

#endif // QTMAIN_H
