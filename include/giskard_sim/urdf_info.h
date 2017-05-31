#pragma once
#include <ros/ros.h>
#include <rviz/panel.h>

#include "giskard_sim/interfaces.h"

namespace Ui
{
class URDFInfo;
}

namespace giskard_sim
{

class URDFInfo: 
  public QWidget, 
  public IScenarioReference, 
  public IURDFListener
{
Q_OBJECT
public:
  URDFInfo( QWidget* parent = 0 );

  ~URDFInfo();

  void setScenario(IScenarioInstance* _pScenario);
  void onURDFChanged(const urdf::Model* model);

protected:
  Ui::URDFInfo *ui_;

private:
  IScenarioInstance* pScenario;

private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void openLoadDialogue();
  void useRelativePath(bool bRelative);
};

}  // giskard_sim
