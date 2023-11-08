/**********************
  开启新的线程用于接受ROS
  消息，不能直接用QT接受
***********************/


#ifndef MESSAGETHREAD_H
#define MESSAGETHREAD_H

#include <QThread>
#include <QMutex>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "robot_msgl/MotorStruct.h"
#include "robot_msgl/TipStruct.h"
#include "robot_msgl/RobotStruct.h"
#include "robot_msgl/ControlStruct.h"

//class QMutex;

class messageThread: public QThread
{
    Q_OBJECT
private:
    QMutex qm;
    bool stop;

public:
    explicit messageThread(QObject *parent = 0);
    void run();
    void SetStopFlg(bool flg);

signals:
    void MainControlStateRecv(robot_msgl::ControlStructConstPtr);

public slots:

    // ROS part
private:
    ros::NodeHandle* pNode;
    ros::Subscriber MainControlStatesub_QT;

private:
    void MainControlStateCallback(const robot_msgl::ControlStruct::ConstPtr& mmsg);
};

#endif // MESSAGETHREAD_H
