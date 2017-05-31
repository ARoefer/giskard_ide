#pragma once
#include <ros/ros.h>
#include <rviz/panel.h>

#include "giskard_sim/interfaces.h"

namespace Ui
{
class ScenarioInfo;
}

namespace giskard_sim
{

class ScenarioInfo:
   public QWidget,
   public IScenarioReference,
   public IScenarioListener
{
Q_OBJECT
public:
  ScenarioInfo( QWidget* parent = 0 );

  ~ScenarioInfo();

  void setScenario(IScenarioInstance* _pScenario);

  std::string getPath();

  void onScenarioLoaded(std::string path, const SScenarioContext* context);

protected:
  Ui::ScenarioInfo *ui_;

private:
  IScenarioInstance* pScenario;

private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void openSaveDialogue();
  void openLoadDialogue();

  void saveScenario();
  void scenarioNameEdited();
};

}  // giskard_sim
