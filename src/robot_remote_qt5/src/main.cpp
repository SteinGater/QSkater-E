#include <QApplication>
#include <QTextCodec>
#include <QSplitter>
#include <QTextEdit>
#include "ros/ros.h"
#include "qtmain.h"

/********************************************************main*****************************************************/
int main(int argc, char *argv[])
{
    // ROS part
    ros::init(argc, argv, "robot_remote");

    QApplication::setStyle("cleanlooks");
    QApplication a(argc, argv);

    QTMain w;
    w.show();

    return a.exec();
}
