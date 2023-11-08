#ifndef CALIBPANEL_H
#define CALIBPANEL_H

#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   

namespace Ui {
class CalibPanel;
}

class CalibPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit CalibPanel(QWidget *parent = nullptr);
  ~CalibPanel();

  virtual void load( const rviz::Config& config )
  {}
  virtual void save( rviz::Config config ) const
  {}

public Q_SLOTS:


private:
  Ui::CalibPanel *ui;
};

#endif // CALIBPANEL_H
