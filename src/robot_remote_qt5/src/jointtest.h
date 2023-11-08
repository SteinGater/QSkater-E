#ifndef JOINTTEST_H
#define JOINTTEST_H

#include <QDialog>
#include <QMessageBox>

#include "ros/ros.h"
#include "robot_msgl/MotorControl.h"


namespace Ui {
class JointTest;
}

class JointTest : public QDialog
{
    Q_OBJECT
    
public:
    explicit JointTest(QWidget *parent = 0);
    ~JointTest();
    
private slots:
    void on_jointGo_clicked();

private:
    Ui::JointTest *ui;

//ROS message recieve part
private:
    ros::NodeHandle* pNode;

    //send the task the robot
    robot_msgl::MotorControl motorcontrolsrv;

    ros::ServiceClient motorclient;

};

#endif // JOINTTEST_H
