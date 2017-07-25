#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>
#include "giskard_sim/scenario_instance.h"
#include "giskard_sim/interfaces.h"

namespace Ui
{
class GiskardSimPanel;
}

namespace rviz {
class InteractiveMarkerDisplay;
}

namespace giskard_sim
{
class RobotModelDisplay;

class GiskardSimPanel: public rviz::Panel, public IErrorListener, public ITopicListener
{
Q_OBJECT
public:
  GiskardSimPanel( QWidget* parent = 0 );

  ~GiskardSimPanel();

  void load( const rviz::Config& config );
  void save( rviz::Config config ) const;
  void onInitialize();
  void onTopicsChanged();

  void onLoadScenarioFailed(const std::string& msg);
  void onLoadURDFFailed(const std::string& msg);
  void onLoadControllerFailed(const std::string& msg);
  void onRunControllerFailed(const std::string& msg);

protected:
  ScenarioInstance scenario;
  ros::NodeHandle nh_;

  Ui::GiskardSimPanel *ui_;
  RobotModelDisplay* modelDisplay;
  rviz::InteractiveMarkerDisplay* intDisplay;

private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.

  void spServiceChanged();
  void commandTopicChanged();
  void jsTopicChanged();
};

}  // giskard_sim
