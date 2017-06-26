#pragma once
#include <ros/ros.h>
#include <rviz/panel.h>
#include "giskard_sim/interfaces.h"

namespace Ui
{
class SimControls;
}

namespace giskard_sim
{

class SimControls: 
  public QWidget, 
  public IScenarioReference,
  public IScenarioListener,
  public IPoseListener
{
Q_OBJECT
public:
  SimControls( QWidget* parent = 0 );

  ~SimControls();

  void setScenario(IScenarioInstance* _pScenario);

  void onScenarioLoaded(std::string path, const SScenarioContext* context);

  void onPoseAdded(std::string pose);
  void onPoseRemoved(std::string pose);
  void onPosesCleared();
  void onPosesLoaded();
  void onPoseRenamed(std::string oldName, std::string newName);

protected:
  Ui::SimControls *ui_;

  void updatePlayPauseBtn(bool bPlay);

private:
  IScenarioInstance* pScenario;

private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void setTimeStep(QString str);
  void timeStepEdited();
  void useTimeStep(bool);

  void setStartPose(QString pose);

  void toggleSim();
  void resetSim();

  // your custom stuff
};

}  // giskard_sim
