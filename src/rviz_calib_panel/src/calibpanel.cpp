#include "calibpanel.h"
#include <QDebug>
#include "ui_calibpanel.h"

CalibPanel::CalibPanel(QWidget *parent) :
  rviz::Panel( parent ),
  ui(new Ui::CalibPanel)
{
  ui->setupUi(this);
}

CalibPanel::~CalibPanel()
{
  delete ui;
}

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(CalibPanel, rviz::Panel )
// END_TUTORIAL

