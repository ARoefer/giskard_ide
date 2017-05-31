#pragma once

#include <QLabel>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include "giskard_sim/interfaces.h"
#include <unordered_map>

namespace Ui
{
class PoseAddJointButton;
class PoseJointItem;
class PosesWidget;
}

using namespace std;

namespace giskard_sim
{
class PoseTreeItem;

class PoseAddJointButton: 
  public QWidget
{
Q_OBJECT
public:
  PoseAddJointButton(PoseTreeItem* parent, const urdf::Model* urdfModel);

  ~PoseAddJointButton();

  QTreeWidgetItem* getItem() const {
    return containerItem;
  }

protected:
  Ui::PoseAddJointButton *ui_;

private:
  PoseTreeItem* poseItem;
  QTreeWidgetItem* containerItem;
  const urdf::Model* urdfModel;

private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void addJoint();  
};

class PoseJointItem: 
  public QWidget
{
Q_OBJECT
public:
  PoseJointItem( PoseTreeItem* parent, string _jointName, double _position, const urdf::Model* _urdfModel);

  ~PoseJointItem();

  QTreeWidgetItem* getItem() const {
    return containerItem;
  }

protected:
  Ui::PoseJointItem *ui_;
  
private:
  PoseTreeItem* poseItem;
  QTreeWidgetItem* containerItem;
  const urdf::Model* urdfModel;
  string jointName;
  double position;

private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void onNameEdited();
  void onPositionEdited();
  void onRemoveJoint();
};


class PoseTreeItem : 
  public QWidget 
{
  Q_OBJECT
public:
  PoseTreeItem(QTreeWidget* parent, IScenarioInstance* _pScenario, string _poseName);

  void setScenario(IScenarioInstance* _pScenario);
  bool renameJoint(string oldName, string newName);
  void setJointPosition(string joint, double pos);
  void addJoint(string joint, double pos);
  void removeJoint(string joint);

  string getPoseName() const {
    return poseName;
  }

  QTreeWidgetItem* getItem() const {
    return containerItem;
  }

private:
  QLineEdit* lePoseName;
  QTreeWidgetItem* containerItem;
  string poseName;
  IScenarioInstance* pScenario;

  unordered_map<string, PoseJointItem*> jointItems;

private Q_SLOTS:
    void onNameEdited();
};

class PosesWidget: 
  public QWidget, 
  public IScenarioReference,
  public IPoseListener
{
Q_OBJECT
public:
  PosesWidget(QWidget* parent = 0);

  ~PosesWidget();

  void setScenario(IScenarioInstance* _pScenario);

  void onPoseAdded(std::string pose);
  void onPoseRemoved(std::string pose);
  void onPosesCleared();
  void onPosesLoaded();
  void onPoseRenamed(std::string oldName, std::string newName);

protected:
  Ui::PosesWidget *ui_;

private:
  IScenarioInstance* pScenario;
  unordered_map<string, PoseTreeItem*> poseItems;

private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void addEmptyPose();
  void setSelectedPose();
  void removeSelectedPose();
  void addCurrentPose();
  void filterURDF(bool bFilter);
};

}  // giskard_sim
