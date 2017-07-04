#pragma once

#include <QWidget>
#include "giskard_sim/datamodel.h"

namespace Ui
{
class SceneWidget;
}

namespace giskard_sim
{

class SceneWidget : public QWidget, public IScenarioReference
{
Q_OBJECT
public:
  SceneWidget( QWidget* parent = 0 );

  ~SceneWidget();

  void setScenario(IScenarioInstance* _pScenario);

public Q_SLOTS:
  void spawnObject(SWorldObject object);

protected:
  Ui::SceneWidget *ui_;

private:
  IScenarioInstance* pScenario;

private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void createObject();
  void deleteObject();
  void selectedObjectChanged();
};

}  // giskard_sim
