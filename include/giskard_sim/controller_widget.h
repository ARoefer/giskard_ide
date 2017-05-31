#pragma once
#include <ros/ros.h>
#include <rviz/panel.h>

#include "giskard_sim/interfaces.h"

#include <QDateTime>
#include <QFileInfo>

#include <unordered_map>

namespace Ui
{
class ControllerWidget;
}

class QFileSystemWatcher;
class QListWidgetItem;
class QTableWidgetItem;

namespace giskard_sim
{

class ControllerWidget: 
  public QWidget, 
  public IScenarioReference, 
  public IControllerListener
{
Q_OBJECT
public:
  ControllerWidget( QWidget* parent = 0 );

  ~ControllerWidget();

  void setScenario(IScenarioInstance* _pScenario);
  void onControllerLoaded(giskard::QPController* controller);
  void onControllerLoadFailed(const std::string& msg);

protected:
  Ui::ControllerWidget *ui_;

private:
  IScenarioInstance* pScenario;
  QFileSystemWatcher* fileWatcher;
  QDateTime lastModified;
  QFileInfo controllerFileInfo;

  std::unordered_map<std::string, QListWidgetItem*> jointItems;

  struct STableRow {
      STableRow() : name(0), type(0), value(0), row(0) {}
      QTableWidgetItem* name;
      QTableWidgetItem* type;
      QTableWidgetItem* value;
      int row;
  };
  std::unordered_map<std::string, STableRow> inputWidgets;

private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void openLoadDialogue();
  void openSaveDialogue();
  void onSaveController();
  void useRelativePath(bool bRelative);
  void onControllerFileChanged();
};

}  // giskard_sim
