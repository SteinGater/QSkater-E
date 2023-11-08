
#include "qtmain.h"
#include "ui_qtmain.h"
#include "jointtest.h"


/*************************************************************************/
QString QTMain::MySetNum(double data)
{
    double redata=1.0*((int)(data/SHOWZERO))*SHOWZERO;
    QString ss;
    return ss.setNum(redata,'g',FLOATPRE);
}

void QTMain::SetFootPosition(double dx,double dy)
{
    double thex=1.0*dx*User_MainBody_ComX;
    double they=1.0*dy*User_MainBody_ComY;
//初始状态
    double my_X[BRANCHN]={User_MainBody_X+thex,User_MainBody_X+thex,-User_MainBody_X+thex,-User_MainBody_X+thex};
    double my_Y[BRANCHN]={-User_MainBody_Y-they,User_MainBody_Y+they,User_MainBody_Y+they,-User_MainBody_Y-they};
    double my_Z[BRANCHN]={-User_MainBody_Z,-User_MainBody_Z,-User_MainBody_Z,-User_MainBody_Z};
    for(int branchi=0;branchi<BRANCHN;branchi++)
    {
        QString InitP;
        InitP=MySetNum(my_X[branchi]);ui->m_leg_position->item(0,branchi)->setText(InitP);
        InitP=MySetNum(my_Y[branchi]);ui->m_leg_position->item(1,branchi)->setText(InitP);
        InitP=MySetNum(my_Z[branchi]);ui->m_leg_position->item(2,branchi)->setText(InitP);
    }
}
void QTMain::SetGaitParameter(double dir,double stepl,double stepa,double stepH,double maginY,double rorateX,double duty,double stepdir,double wheelangle)
{
    QString Parameter;
    Parameter=MySetNum(dir);ui->m_gait_parament->item(0,0)->setText(Parameter);
    Parameter=MySetNum(stepl);ui->m_gait_parament->item(0,1)->setText(Parameter);
    Parameter=MySetNum(stepa);ui->m_gait_parament->item(0,2)->setText(Parameter);
    Parameter=MySetNum(stepH);ui->m_gait_parament->item(0,3)->setText(Parameter);
    Parameter=MySetNum(maginY);ui->m_gait_parament2->item(0,0)->setText(Parameter);
    Parameter=MySetNum(rorateX);ui->m_gait_parament2->item(0,1)->setText(Parameter);
    Parameter=MySetNum(duty);ui->m_gait_parament2->item(0,2)->setText(Parameter);
    Parameter=MySetNum(stepdir);ui->m_gait_parament2->item(0,3)->setText(Parameter);
    Parameter=MySetNum(wheelangle);ui->m_gait_parament2->item(0,4)->setText(Parameter);
}
/*************************************************************************/
QTMain::QTMain(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::QTMain)
{
    ui->setupUi(this);

    pNode  =new ros::NodeHandle;

    pMythread =  new messageThread();
    pMythread->start();

    //receive the state of robot
    qRegisterMetaType<robot_msgl::ControlStructConstPtr>("robot_msgl::ControlStructConstPtr");

    //transform to the new function
    connect(pMythread,SIGNAL(MainControlStateRecv(robot_msgl::ControlStructConstPtr)),this,SLOT(handleMainControlStateChange(robot_msgl::ControlStructConstPtr)));

    //send the state of robot
    motorclient=pNode->serviceClient<robot_msgl::MotorControl>("robot_msgl/MotorControl");
    bodyclient=pNode->serviceClient<robot_msgl::RobotCommand>("robot_msgl/RobotCommand");
    computeclient=pNode->serviceClient<robot_msgl::ComputeControl>("robot_msgl/ComputeControl");

    //初始标志
    the_connect_flag=0;
    RobotState.Simulation_Flag=SIMULATIONWORLD;
    the_open_3dros_flag=0;

    //初始模式直接链接main
    on_actionAutopan_triggered();
    //////////////////////
    //初始状态
    SetFootPosition(0,0);
    QStringList ModelList;
    QStringList TypeList;
    for(int thei=0;thei<BRANCH_MODELN;thei++)
    {
        ModelList<<BRANCH_MODEL_NAME[thei];
    }
    for(int branchi=0;branchi<BRANCHN;branchi++)
    {
        ui->m_leg_position->item(3,branchi)->setText("0");
        ui->m_leg_position->item(4,branchi)->setText("0");
        ui->m_leg_position->item(5,branchi)->setText("0");
        ui->m_leg_position->item(7,branchi)->setText("1");
        BranchiModelBox[branchi].addItems(ModelList);
        ui->m_leg_position->setCellWidget(6,branchi,(QWidget*)(&BranchiModelBox[branchi]));
    }


    //刷新页面定时器
    qtreviewtime=startTimer(CONTROLER_INTERVAL);

    p_state_bar=new QLabel("   ",this);
    ui->statusbar->addPermanentWidget(p_state_bar);
    this->resize(this->minimumSize());
    ui->dockWidget_rviz->setWindowFlag(Qt::FramelessWindowHint);
    ui->dockWidget_rviz->close();
    /****rviz***/
    render_panel = new rviz::RenderPanel;
    ui->rviz_show->addWidget(render_panel);
    rviz_manager=new rviz::VisualizationManager(render_panel);
    render_panel->initialize(rviz_manager->getSceneManager(),rviz_manager);
    rviz_manager->initialize();
    rviz_manager->removeAllDisplays();
    rviz::YamlConfigReader rviz_reader;
    rviz_reader.readFile( rviz_config, "../QuadrupedRobotCOI/src/robot_3dros/urdf.rviz");
    rviz_manager->load(rviz_config.mapGetChild("Visualization Manager"));
    rviz_manager->startUpdate();


}

QTMain::~QTMain()
{
    if(the_open_3dros_flag==1)
    {
        system("rosnode kill robot_3dros &");
        system("rosnode kill robot_state_publisher &");
    }
    //system("rosnode kill rviz &");
    ros::shutdown();
    delete render_panel;
    delete rviz_manager;
    delete ui;
    this->close();
}

//**********************************************************************************the time of changing the show state
void QTMain::timerEvent(QTimerEvent *event)
{
    if(event->timerId()==qtreviewtime)
    {
        QTableWidgetItem *pItem;
        //状态栏提示 主控制状态=机身状态
        QString state_bar_str;
        if(the_connect_flag)
        {
            state_bar_str=tr("主控：");
            if(RobotState.Simulation_Flag==REALWORLD){state_bar_str=state_bar_str+tr("真实运动|");}
            else{state_bar_str=state_bar_str+tr("虚拟仿真|");}
            if(RobotState.Robot_State<0)
            {
                state_bar_str=state_bar_str+tr("机器人急停状态");
            }
            else
            {
                state_bar_str=state_bar_str+ui->m_maincommand->itemText(RobotState.Robot_State);
            }
            state_bar_str=state_bar_str+tr("|");
            int theindex=RobotState.Control_Model;
            if(theindex==2)
            {
                if(RobotState.RealRobot.Robot_Model==1)
                {
                    theindex=3;
                }
            }
            else if(theindex==3)
            {
                theindex=13;
            }
            else if(theindex==10)
            {
                theindex=13;
            }
            else if(theindex==100)
            {
                theindex=1;
            }
            state_bar_str=state_bar_str+CONTROL_MODEL[RobotState.Control_Model];
            state_bar_str=state_bar_str+tr("|");
            state_bar_str=state_bar_str+GAIT_TYPE[RobotState.Gait_Type];
        }
        else
        {
            state_bar_str=tr("主控未连接");
        }
        p_state_bar->setText(state_bar_str);
        //显示一些错误
        if(RobotState.design_error)
        {
            ui->Gait_design_error->setText(Gait_Design_Error[RobotState.design_error]+tr("错误"));
        }
        if(RobotState.body_error)
        {
            ui->Body_error->setText(tr("超出")+Robot_Body_Error[RobotState.body_error]);
        }
        //显示机身数据
        QString str;QString str_real;QString str_exp;
        ui->BodyState->clearContents();
        for(int i=0;i<3;i++)
        {
            int j=i+3;
            str_real=MySetNum(RobotState.RealRobot.body.P[i]);str_exp=MySetNum(RobotState.ExpRobot.body.P[i]);
            str=str_real+tr("/")+str_exp;pItem = new QTableWidgetItem(str);ui->BodyState->setItem(0,j,pItem);
            str_real=MySetNum(RobotState.RealRobot.body.Rw[i]);str_exp=MySetNum(RobotState.ExpRobot.body.Rw[i]);
            str=str_real+tr("/")+str_exp;pItem = new QTableWidgetItem(str);ui->BodyState->setItem(0,i,pItem);
        }
        for(int i=0;i<6;i++)
        {
            str_real=MySetNum(RobotState.RealRobot.body.V[i]);str_exp= MySetNum(RobotState.ExpRobot.body.V[i]);
            str=str_real+tr("/")+str_exp;pItem = new QTableWidgetItem(str);ui->BodyState->setItem(1,i,pItem);
            str_real=MySetNum(RobotState.RealRobot.body.A[i]);str_exp= MySetNum(RobotState.ExpRobot.body.A[i]);
            str=str_real+tr("/")+str_exp;pItem = new QTableWidgetItem(str);ui->BodyState->setItem(2,i,pItem);
            str_real=MySetNum(RobotState.RealRobot.body.F[i]);str_exp= MySetNum(RobotState.ExpRobot.body.F[i]);
            str=str_real+tr("/")+str_exp;pItem = new QTableWidgetItem(str);ui->BodyState->setItem(3,i,pItem);
        }
        //显示腿状态
        ui->LegState->clearContents();
        for(int i=0;i<BRANCHN;i++)
        {
            str=BRANCH_MODEL_NAME[RobotState.RealRobot.BranchModel[i]]+tr("/")+BRANCH_MODEL_NAME[RobotState.ExpRobot.BranchModel[i]];
            pItem = new QTableWidgetItem(str);ui->LegState->setItem(0,i,pItem);
            //str=BRANCH_STATE_NAME[RobotState.BranchModel[i]][RobotState.BranchState[i]]+tr("/")+BRANCH_STATE_NAME[RobotState.ExpRobot.BranchModel[i]][RobotState.ExpRobot.BranchState[i]];
            str_real=MySetNum(RobotState.RealRobot.BranchState[i]);str_exp=MySetNum(RobotState.ExpRobot.BranchState[i]);
            str=str_real+tr("/")+str_exp;pItem = new QTableWidgetItem(str);ui->LegState->setItem(1,i,pItem);
            for(int j=0;j<3;j++)
            {
                int k0=j+3;int k1=j+2;int k2=j+5;int k3=j+8;
                str_real= MySetNum(RobotState.RealRobot.BranchTip[i].P[j]);str_exp= MySetNum(RobotState.ExpRobot.BranchTip[i].P[j]);
                str=str_real+tr("/")+str_exp;pItem = new QTableWidgetItem(str);ui->LegState->setItem(k1,i,pItem);
                str_real= MySetNum(RobotState.RealRobot.BranchTip[i].V[k0]);str_exp= MySetNum(RobotState.ExpRobot.BranchTip[i].V[k0]);
                str=str_real+tr("/")+str_exp;pItem = new QTableWidgetItem(str);ui->LegState->setItem(k2,i,pItem);
                str_real= MySetNum(RobotState.RealRobot.BranchTip[i].F[k0]);str_exp= MySetNum(RobotState.ExpRobot.BranchTip[i].F[k0]);
                str=str_real+tr("/")+str_exp;pItem = new QTableWidgetItem(str);ui->LegState->setItem(k3,i,pItem);
            }
        }
        //显示关节
        ui->AngleState->clearContents();
        SumForce=0;Sumerror=0;SumAble=0;
        for(int i=0;i<(BRANCHN*BRANCH_BODYN);i++)
        {
            int j=i+1;
            str= MySetNum(RobotState.RealRobot.MotorState[i].ID);pItem=ui->AngleState->horizontalHeaderItem(j);pItem->setText(str);
            str_real=MySetNum(RobotState.RealRobot.MotorState[i].angle);str_exp=MySetNum(RobotState.ExpRobot.MotorState[i].angle);
            str=str_real+tr("/")+str_exp;pItem = new QTableWidgetItem(str);ui->AngleState->setItem(0,j,pItem);
            str_real=MySetNum(RobotState.RealRobot.MotorState[i].velocity);str_exp=MySetNum(RobotState.ExpRobot.MotorState[i].velocity);
            str=str_real+tr("/")+str_exp;pItem = new QTableWidgetItem(str);ui->AngleState->setItem(1,j,pItem);
            str_real=MySetNum(RobotState.RealRobot.MotorState[i].accelation);str_exp=MySetNum(RobotState.ExpRobot.MotorState[i].accelation);
            str=str_real+tr("/")+str_exp;pItem = new QTableWidgetItem(str);ui->AngleState->setItem(2,j,pItem);
            str_real=MySetNum(RobotState.RealRobot.MotorState[i].torque);str_exp=MySetNum(RobotState.ExpRobot.MotorState[i].torque);
            str=str_real+tr("/")+str_exp;pItem = new QTableWidgetItem(str);ui->AngleState->setItem(3,j,pItem);
            str_real=MySetNum(RobotState.RealRobot.MotorState[i].Enable);str_exp=MySetNum(RobotState.ExpRobot.MotorState[i].Enable);
            str=str_real+tr("/")+str_exp;pItem = new QTableWidgetItem(str);ui->AngleState->setItem(4,j,pItem);
            str= MySetNum(RobotState.RealRobot.MotorState[i].error);pItem = new QTableWidgetItem(str);ui->AngleState->setItem(5,j,pItem);
            SumForce=SumForce+fabs(RobotState.RealRobot.MotorState[i].torque);
            SumAble=SumAble+RobotState.RealRobot.MotorState[i].Enable;
            Sumerror=Sumerror+RobotState.RealRobot.MotorState[i].error;
        }
        str= "Sum";pItem=ui->AngleState->horizontalHeaderItem(0);pItem->setText(str);
        str= "0";pItem = new QTableWidgetItem(str);ui->AngleState->setItem(0,0,pItem);
        str= MySetNum(SumForce);pItem = new QTableWidgetItem(str);ui->AngleState->setItem(2,0,pItem);
        str= MySetNum(SumAble);pItem = new QTableWidgetItem(str);ui->AngleState->setItem(4,0,pItem);
        str= MySetNum(Sumerror);pItem = new QTableWidgetItem(str);ui->AngleState->setItem(6,0,pItem);
    }
    //保持链接状态
    the_connect_time=the_connect_time+1;
    if(the_connect_time>=CONNECT_TIME)
    {
        the_connect_flag=0;
    }
}

//***********************************************************************************************ROS message recieve part
void QTMain::handleMainControlStateChange(robot_msgl::ControlStructConstPtr mmsg)
{
    //contact the robot
    the_connect_flag=1;
    the_connect_time=0;
    RobotState=*mmsg;
}
/***************************************************the function of ui*************************************************/
/////////////////////////////////////////////the function of manubar
//模式选择手动模式
void QTMain::on_actionManualpan_triggered()
{
    //改变标志位
    the_mode_connect=0;

    //改变控制菜单兰
    ui->stackedWidget->setCurrentIndex(1);
    //ui->manual_pan->setVisible(true);
    //ui->auto_pan->close();
}

//模式选择自动模式
void QTMain::on_actionAutopan_triggered()
{
    //改变标志位
    the_mode_connect=1;

    //改变控制菜单兰
    ui->stackedWidget->setCurrentIndex(0);
    //ui->manual_pan->close();
    //ui->auto_pan->setVisible(true);
}

//关节测试
void QTMain::on_actionJointtest_triggered()
{
    JointTest *myjointtest=new JointTest(this);
    myjointtest->setModal(false);
    myjointtest->show();
}

//open the Gazebo
void QTMain::on_actionGazebo_triggered()
{
    system("gnome-terminal -x bash -c 'source .bashrc;roslaunch robot_3dros robot_gazebo.launch' ");
}

//base send the joint
void QTMain::send_the_joint_base(int ID,int Command,double value)
{
    motorcontrolsrv.request.ID=ID;
    motorcontrolsrv.request.Command=Command;
    motorcontrolsrv.request.value=value;
    if (motorclient.call(motorcontrolsrv))
    {
        //QMessageBox::warning(this,tr("成功信息"),tr("成功发送指令并执行"));
    }
    else
    {
        QMessageBox::warning(this,tr("失败信息"),tr("发送服务失败，未能链接主机"));
    }
}
//send the clear error
void QTMain::on_actionClearAllError_triggered()
{
    if(the_connect_flag){send_the_joint_base(0,MOTORCOMMAND_CLEAR,0);}
}
//send the torque on
void QTMain::on_actionTORQUE_ON_triggered()
{
    if(the_connect_flag){send_the_joint_base(0,MOTORCOMMAND_ENABLE,1);}
}
//send the torque off
void QTMain::on_actionTORQUE_OFF_triggered()
{
    if(the_connect_flag){send_the_joint_base(0,MOTORCOMMAND_ENABLE,0);}
}

//the base send the compute command
void QTMain::send_the_compute_base(int Command)
{
    computecontrolsrv.request.computeCommand=Command;
    if(computeclient.call(computecontrolsrv))
    {
        //QMessageBox::warning(this,tr("成功信息"),tr("成功发送指令并执行"));
    }
    else
    {
        QMessageBox::warning(this,tr("失败信息"),tr("发送服务失败，未能链接主机"));
    }
}
void QTMain::on_actionSIMULATE_triggered()
{
    if(the_connect_flag){send_the_compute_base(computecontrolsrv.request.COMPUTE_SIMULATE);}
}
void QTMain::on_actionREALWORD_triggered()
{
    if(the_connect_flag){send_the_compute_base(computecontrolsrv.request.COMPUTE_REALWORD);}
}
void QTMain::on_actionOPENIMU_triggered()
{
    if(the_connect_flag){send_the_compute_base(computecontrolsrv.request.COMPUTE_IMUOPEN);}
}
void QTMain::on_actionCLOSEIMU_triggered()
{
    if(the_connect_flag){send_the_compute_base(computecontrolsrv.request.COMPUTE_IMUCLOSE);}
}
void QTMain::on_actionPosition_triggered()
{
    if(the_connect_flag){send_the_compute_base(computecontrolsrv.request.COMPUTE_JOINT_POSITION);}
}
void QTMain::on_actionVelocity_triggered()
{
    if(the_connect_flag){send_the_compute_base(computecontrolsrv.request.COMPUTE_JOINT_VELOCITY);}
}
void QTMain::on_actionTorque_triggered()
{
    if(the_connect_flag){send_the_compute_base(computecontrolsrv.request.COMPUTE_JOINT_TORQUE);}
}
void QTMain::on_actionReset_triggered()
{
    if(the_connect_flag){send_the_compute_base(computecontrolsrv.request.COMPUTE_RESET_JOINT);}
}
/////////////////////////////////////////////the function of manual
void QTMain::Mani_Read_Data(void)
{
    robotcommadsrv.request.Robot_Command=ui->m_maincommand->currentIndex();
    if((robotcommadsrv.request.Robot_Command>=2)&&(robotcommadsrv.request.Robot_Command<=5))
    {
        robotcommadsrv.request.Robot_Command=2;
        robotcommadsrv.request.gait_type=ui->m_gait_type->currentIndex();
    }
    else if((robotcommadsrv.request.Robot_Command>=7)&&(robotcommadsrv.request.Robot_Command<7))
    {
        robotcommadsrv.request.Robot_Command=2;
        robotcommadsrv.request.gait_type=11;
    }
    else if((robotcommadsrv.request.Robot_Command>=8)&&(robotcommadsrv.request.Robot_Command<=9))
    {
        robotcommadsrv.request.Robot_Command=2;
        robotcommadsrv.request.gait_type=10;
    }
    else if((robotcommadsrv.request.Robot_Command>=10)&&(robotcommadsrv.request.Robot_Command<=12))
    {
        robotcommadsrv.request.Robot_Command=10;
        robotcommadsrv.request.gait_type=ui->m_gait_type->currentIndex();
    }
    else if(robotcommadsrv.request.Robot_Command==13)
    {
        robotcommadsrv.request.Robot_Command=3;
        robotcommadsrv.request.gait_type=ui->m_gait_type->currentIndex();
    }
    //get body
    for(int i=0;i<3;i++)
    {
        int j=i+3;
        robotcommadsrv.request.body.P[i]=ui->m_exp->item(0,i)->text().toDouble();
        robotcommadsrv.request.body.Rw[i]=ui->m_exp->item(0,j)->text().toDouble();
    }
    //get branch
    for(int i=0;i<BRANCHN;i++)
    {
        for(int j=0;j<3;j++)
        {
            int k=j+3;
            robotcommadsrv.request.BranchTip[i].P[j]=ui->m_leg_position->item(j,i)->text().toDouble();
            robotcommadsrv.request.BranchTip[i].Rw[j]=ui->m_leg_position->item(k,i)->text().toDouble();
        }
        robotcommadsrv.request.BranchModel[i]=BranchiModelBox[i].currentIndex();
        robotcommadsrv.request.BranchState[i]=ui->m_leg_position->item(7,i)->text().toInt();
    }
    robotcommadsrv.request.time[0]=ui->m_time->item(0,0)->text().toDouble();
    robotcommadsrv.request.time[1]=ui->m_time->item(0,1)->text().toDouble();
    for(int ii=0;ii<4;ii++)
    {
        robotcommadsrv.request.gait_para[ii]=ui->m_gait_parament->item(0,ii)->text().toDouble();
    }
    for(int ii=0;ii<5;ii++)
    {
        robotcommadsrv.request.skating_para[ii]=ui->m_gait_parament2->item(0,ii)->text().toDouble();
    }
}
void QTMain::Mani_Send_Command(void)
{
    if (bodyclient.call(robotcommadsrv))
    {
        if(robotcommadsrv.response.error)
        {
            QString errorinformation=tr("指令未成功执行，错误!!!");
            QMessageBox::warning(this,tr("失败信息"),errorinformation);
        }
        else
        {
            //ui->statusbar->showMessage(tr("成功发送指令并执行"),1000);
        }
    }
    else
    {
        QMessageBox::warning(this,tr("失败信息"),tr("发送服务失败，未能链接主机"));
    }
}
void QTMain::SetParameter(int command,int gait_type)
{
    if(command==1)//stay
    {
        ui->m_time->item(0,0)->setText("1");
        ui->m_time->item(0,1)->setText("1");
        SetGaitParameter(0,0,0,0.05,0,0,0.5,0,0);
        for(int ii=0;ii<BRANCHN;ii++){BranchiModelBox[ii].setCurrentIndex(RobotState.RealRobot.BranchModel[ii]);}
    }
    else if(command==2)//leg open move
    {
        ui->m_time->item(0,0)->setText("0.6");
        ui->m_time->item(0,1)->setText("4");
        SetGaitParameter(0,0,0,0.05,0,0,0.5,0,0);
        SetFootPosition(1,0);
        for(int ii=0;ii<BRANCHN;ii++){BranchiModelBox[ii].setCurrentIndex(0);}
    }
    else if(command==3)//wheel open move
    {
        ui->m_time->item(0,0)->setText("0.6");
        ui->m_time->item(0,1)->setText("4");
        if(gait_type)
        {
            SetGaitParameter(0,0,0,0.03,0.04,0,0.5,0,M_PI/9);
            SetFootPosition(-1,-1);
        }
        else
        {
            SetGaitParameter(0,0,0,0.05,0,0,0.5,0,M_PI/9);
            SetFootPosition(-1,0);
        }
        for(int ii=0;ii<BRANCHN;ii++){BranchiModelBox[ii].setCurrentIndex(1);}
    }
    else if(command==4)//leg adapt move
    {
        ui->m_time->item(0,0)->setText("0.3");
        ui->m_time->item(0,1)->setText("4");
        SetGaitParameter(0,0,0, 0.05, 0,0,0.5,0,0);
        SetFootPosition(1,0);
        for(int ii=0;ii<BRANCHN;ii++){BranchiModelBox[ii].setCurrentIndex(0);}
    }
    else if(command==5)//wheel adapt move
    {
        if(gait_type)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
        {
            ui->m_time->item(0,0)->setText("0.9");
            ui->m_time->item(0,1)->setText("4");
            SetGaitParameter(0,0.03,0,0.02,0.08,0.1,0.7,0,M_PI/9);
            SetFootPosition(-1,-1);
        }
        else
        {
            ui->m_time->item(0,0)->setText("0.6");
            ui->m_time->item(0,1)->setText("4");
            SetGaitParameter(0,0,0,0.05,0.0,0.0,0.5,0,M_PI/9);
            SetFootPosition(-1,0);
        }
        for(int ii=0;ii<BRANCHN;ii++){BranchiModelBox[ii].setCurrentIndex(1);}
    }
    else if(command==6)//leg static move
    {

    }
    else if(command==7)//wheel static move
    {
        ui->m_time->item(0,0)->setText("2.0");
        ui->m_time->item(0,1)->setText("4");
        SetGaitParameter(0,0.05,0, 0.0, 0.0,0.0,0.0,0.0,M_PI/4);
        SetFootPosition(-1,0);
        for(int ii=0;ii<BRANCHN;ii++){BranchiModelBox[ii].setCurrentIndex(1);}
    }
    else if((command>=10)&&(command<=12))//exchange
    {
        ui->m_time->item(0,0)->setText("4");
        ui->m_time->item(0,1)->setText("1");
        SetGaitParameter(0,0,0,0.05,0,0,0.5,0,0);
        SetFootPosition(0,0);
        if(command<=11)
        {
            for(int ii=0;ii<BRANCHN;ii++){BranchiModelBox[ii].setCurrentIndex(11-command);}
        }
    }
    else if(command==13)
    {
        ui->m_exp->item(0,2)->setText("0.3");
        ui->m_time->item(0,0)->setText("0.1");
        ui->m_time->item(0,1)->setText("1");
        SetGaitParameter(0,0,0,0.05,0,0,0.5,0,0);
        SetFootPosition(0,0);
        for(int ii=0;ii<BRANCHN;ii++){BranchiModelBox[ii].setCurrentIndex(RobotState.RealRobot.BranchModel[ii]);}
    }
    else if(command>=100)//show
    {
        ui->m_time->item(0,0)->setText("1");
        ui->m_time->item(0,1)->setText("1");
        SetGaitParameter(0,0,0,0.05,0,0,0.5,0,0);
        for(int ii=0;ii<BRANCHN;ii++){BranchiModelBox[ii].setCurrentIndex(RobotState.RealRobot.BranchModel[ii]);}
    }
}

void QTMain::on_m_maincommand_currentIndexChanged(int index)
{
    if(index==1)//stay
    {
        ui->m_gait_type->setCurrentIndex(0);
        SetParameter(index,0);
    }
    else if(index==2)//leg open move
    {
        ui->m_gait_type->setCurrentIndex(0);
        SetParameter(index,0);
    }
    else if(index==3)//wheel open move
    {
        ui->m_gait_type->setCurrentIndex(1);
        SetParameter(index,1);
    }
    else if(index==4)//leg adapt move
    {
        ui->m_gait_type->setCurrentIndex(0);
        SetParameter(index,0);
    }
    else if(index==5)//wheel adapt move
    {
        ui->m_gait_type->setCurrentIndex(1);
        SetParameter(index,1);
    }
    else if(index==6)//leg statci move
    {

    }
    else if(index==7)//wheel static move
    {
        ui->m_gait_type->setCurrentIndex(1);
        SetParameter(index,1);
    }
    else if((index>=10)&&(index<=12))//exchange
    {
        ui->m_gait_type->setCurrentIndex(0);
        SetParameter(index,0);
    }
    else if(index==13)
    {
        ui->m_gait_type->setCurrentIndex(0);
        SetParameter(index,0);
    }
    else if(index>=100)//show
    {
        ui->m_gait_type->setCurrentIndex(0);
        SetParameter(index,0);
    }
    else
    {

    }
}
void QTMain::on_m_gait_type_currentIndexChanged(int index)
{
    SetParameter(ui->m_maincommand->currentIndex(),index);
}
void QTMain::on_actionStandup_triggered()
{
    Mani_Read_Data();
    robotcommadsrv.request.Robot_Command=12;
    robotcommadsrv.request.time[0]=4;
    if(the_connect_flag)
    {
        Mani_Send_Command();
    }
}
void QTMain::on_actionLaydown_triggered()
{
    robotcommadsrv.request.Robot_Command=12;
    robotcommadsrv.request.body.P[0]=0;
    robotcommadsrv.request.body.P[1]=0;
    robotcommadsrv.request.body.P[2]=0.11;
    robotcommadsrv.request.body.Rw[0]=0;
    robotcommadsrv.request.body.Rw[1]=0;
    robotcommadsrv.request.body.Rw[2]=0;
    double thedX=0.015;
    double my_X[BRANCHN]={User_MainBody_X+thedX,User_MainBody_X+thedX,-User_MainBody_X-thedX,-User_MainBody_X-thedX};
    double my_Y[BRANCHN]={-User_MainBody_Y,User_MainBody_Y,User_MainBody_Y,-User_MainBody_Y};
    double my_Z[BRANCHN]={-User_MainBody_Z,-User_MainBody_Z,-User_MainBody_Z,-User_MainBody_Z};
    double my_model[BRANCHN]={1,1,0,0};
    for(int i=0;i<BRANCHN;i++)
    {
        robotcommadsrv.request.BranchTip[i].P[0]=my_X[i];
        robotcommadsrv.request.BranchTip[i].P[1]=my_Y[i];
        robotcommadsrv.request.BranchTip[i].P[2]=my_Z[i];
        for(int j=0;j<3;j++)
        {
            robotcommadsrv.request.BranchTip[i].Rw[j]=0;
        }
        robotcommadsrv.request.BranchModel[i]=my_model[i];
        robotcommadsrv.request.BranchState[i]=LEG_SUP_STATE;
    }
    robotcommadsrv.request.time[0]=4;
    if(the_connect_flag)
    {
        Mani_Send_Command();
    }
}
//send the manual function
void QTMain::on_m_send_command_clicked()
{
    Mani_Read_Data();
    if(the_connect_flag)
    {
        Mani_Send_Command();
    }
}
//send the stop fast
void QTMain::on_m_send_stop_clicked()
{
    robotcommadsrv.request.Robot_Command=MAIN_STOP;
    if(the_connect_flag)
    {
        Mani_Send_Command();
    }
}

/***********************************************easy move***/
void QTMain::send_easymove_base(double easy_dir[3])
{
    Mani_Read_Data();
    robotcommadsrv.request.gait_para[0]=easy_dir[0];
    robotcommadsrv.request.gait_para[1]=easy_dir[1];
    robotcommadsrv.request.gait_para[2]=easy_dir[2];
    if(RobotState.RealRobot.Robot_Model==1)
    {
        robotcommadsrv.request.time[0]=1.2;
        robotcommadsrv.request.Robot_Command=4;
    }
    else
    {
        robotcommadsrv.request.time[0]=0.8;
        robotcommadsrv.request.Robot_Command=3;
    }
    bodyclient.call(robotcommadsrv);
}
void QTMain::on_a_easybodyshow_clicked()
{
    Mani_Read_Data();
    robotcommadsrv.request.Robot_Command=100;
    if(the_connect_flag)
    {
        Mani_Send_Command();
    }
}
void QTMain::on_a_easyfront_clicked()
{
    double easy_d[3]={0,0.1,0};
    send_easymove_base(easy_d);
}
void QTMain::on_a_easystop_clicked()
{
    double easy_d[3]={0,0,0};
    send_easymove_base(easy_d);
}
void QTMain::on_a_easyback_clicked()
{
    double easy_d[3]={0,-0.1,0};
    send_easymove_base(easy_d);
}
void QTMain::on_a_easyleft_clicked()
{
    double easy_d[3]={M_PI/2,-0.1,0};
    send_easymove_base(easy_d);
}

void QTMain::on_a_easyright_clicked()
{
    double easy_d[3]={M_PI/2,0.1,0};
    send_easymove_base(easy_d);
}
void QTMain::on_a_easylrorate_clicked()
{
    double easy_d[3]={0,0,0.1};
    send_easymove_base(easy_d);
}

void QTMain::on_a_easyrrorate_clicked()
{
    double easy_d[3]={0,0,-0.1};
    send_easymove_base(easy_d);
}

void QTMain::on_pushButton_rviz_show_clicked()
{
    if(ui->dockWidget_rviz->isHidden())
        ui->dockWidget_rviz->show();
    else
        ui->dockWidget_rviz->close();
}

void QTMain::on_dockWidget_rviz_visibilityChanged(bool visible)
{
    QSize rviz_hiden_min_size(1000, 673);
    QSize rviz_visible_min_size(1000, 673 + ui->dockWidget_rviz->minimumHeight());
    if(!visible)
    {
        this->setMinimumSize(rviz_hiden_min_size);
        this->resize(rviz_hiden_min_size);
        this->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
    }
    else
    {
        this->setMinimumSize(rviz_visible_min_size);
        this->resize(rviz_visible_min_size);
        this->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Expanding);
    }

}
