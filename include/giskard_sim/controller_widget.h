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
  void onControllerLoaded(giskard_core::QPController* controller);
  void onControllerLoadFailed(const std::string& msg);

  void onInputAssignmentChanged(boost::shared_ptr<IInputAssignment> assignment);
  void onInputAssignmentDeleted(const std::string& inputName);

protected:
  Ui::ControllerWidget *ui_;

private:
  IScenarioInstance* pScenario;
  QFileSystemWatcher* fileWatcher;
  QDateTime lastModified;
  QFileInfo controllerFileInfo;

  std::unordered_map<std::string, QListWidgetItem*> jointItems;

  struct SInputItem {
    QListWidgetItem* item;
    QWidget* inputWidget;
  };
  std::unordered_map<std::string, SInputItem> inputWidgets;

private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void openLoadDialogue();
  void openSaveDialogue();
  void onSaveController();
  void useRelativePath(bool bRelative);
  void onControllerFileChanged();
  void assignmentChanged(boost::shared_ptr<IInputAssignment> value);
};

}  // giskard_sim
