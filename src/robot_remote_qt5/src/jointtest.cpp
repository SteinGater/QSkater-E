#include "jointtest.h"
#include "ui_jointtest.h"

JointTest::JointTest(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::JointTest)
{
    ui->setupUi(this);
    pNode  =new ros::NodeHandle;

    //send the state of robot
    motorclient=pNode->serviceClient<robot_msgl::MotorControl>("robot_msgl/MotorControl");
}

JointTest::~JointTest()
{
    delete ui;
}


// send the joint command
void JointTest::on_jointGo_clicked()
{
    motorcontrolsrv.request.ID=ui->jointID->value();
    motorcontrolsrv.request.Command=ui->jointcommand->currentIndex();
    motorcontrolsrv.request.value=ui->jointangle->value();
    if (motorclient.call(motorcontrolsrv))
    {
    }
    else
    {
        QMessageBox::warning(this,tr("失败信息"),tr("发送服务失败，未能链接主机"));
    }
}
