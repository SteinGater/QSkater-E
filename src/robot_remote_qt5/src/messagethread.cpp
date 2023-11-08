#include<QDebug>
#include<QMutexLocker>
#include <QTableWidget>

#include "qtmain.h"

#include "messagethread.h"

messageThread::messageThread(QObject *parent) :
    QThread(parent)
{
    stop = false;
    pNode  =new ros::NodeHandle;
    MainControlStatesub_QT = pNode->subscribe<robot_msgl::ControlStruct>("robot_msgl/ControlStruct", 1, &messageThread::MainControlStateCallback, this);

}

void messageThread::SetStopFlg(bool flg)
{
    QMutexLocker locker(&qm);
    stop=flg;
}

void messageThread::run()
{
    ros::spin();
    stop = false;
}

void messageThread::MainControlStateCallback(const robot_msgl::ControlStruct::ConstPtr& mmsg)
{
    emit MainControlStateRecv(mmsg);
}

