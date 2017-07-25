#pragma once

#include <QWidget>
#include "giskard_sim/datamodel.h"

namespace Ui
{
class SceneWidget;
}

namespace rviz {
class FrameManager;
}

namespace giskard_sim
{

class SceneWidget : public QWidget, public IScenarioReference, public IScenarioListener
{
Q_OBJECT
public:
  SceneWidget( QWidget* parent = 0 );

  ~SceneWidget();

  void setScenario(IScenarioInstance* _pScenario);
  void setFrameManager(rviz::FrameManager* frameManager);

  void onScenarioLoaded(std::string path, const SScenarioContext* context);
  void onObjectAdded(const SWorldObject& object);
  void onObjectChanged(const SWorldObject& object);
  void onObjectRemoved(const std::string& name);
  void onSelectedObjectChanged(const std::string& selected);

public Q_SLOTS:
  void spawnObject(SWorldObject object);

protected:
  Ui::SceneWidget *ui_;
  rviz::FrameManager* frameManager;

private:
  IScenarioInstance* pScenario;
  std::string selected;

private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void createObject();
  void deleteObject();
  void selectedObjectChanged();
  void objectChanged(giskard_sim::SWorldObject object);
};

}  // giskard_sim
