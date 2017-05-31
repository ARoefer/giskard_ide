#include "giskard_sim/pose_tree_widget.h"
#include "giskard_sim/datamodel.h"

#include "ui_poses_widget.h"
#include "ui_pose_joint_item.h"
#include "ui_pose_add_joint_item.h"

#include <urdf/model.h>

namespace giskard_sim {

PoseAddJointButton::PoseAddJointButton(PoseTreeItem* parent, const urdf::Model* _urdfModel)
  : QWidget( parent )
  , poseItem(parent)
  , ui_(new Ui::PoseAddJointButton())
  , urdfModel(_urdfModel)
{
  // set up the GUI
  ui_->setupUi(this);

  containerItem = new QTreeWidgetItem(poseItem->getItem());
  containerItem->treeWidget()->setItemWidget(containerItem, 0, this);
}

PoseAddJointButton::~PoseAddJointButton() {
  delete ui_;
}

void PoseAddJointButton::addJoint() {
  std::cout << "Add joint was pressed..." << std::endl;
  std::cout << "TODO: Open Menu showing joints" << std::endl;
}


PoseJointItem::PoseJointItem(PoseTreeItem* parent, string _jointName, double _position, const urdf::Model* _urdfModel)
  : QWidget( parent )
  , poseItem(parent)
  , ui_(new Ui::PoseJointItem())
  , jointName(_jointName)
  , position(_position)
  , urdfModel(_urdfModel)
{
  // set up the GUI
  ui_->setupUi(this);

  ui_->leJointPos->setValidator(new QDoubleValidator(-10000.0, -10000.0, 8, this));

  ui_->leJointName->setText(QString::fromStdString(jointName));
  ui_->leJointPos->setText(QString::number(position));

  containerItem = new QTreeWidgetItem(poseItem->getItem());
  containerItem->treeWidget()->setItemWidget(containerItem, 0, this);
}

PoseJointItem::~PoseJointItem() {
  delete ui_;
}

void PoseJointItem::onNameEdited() {
	string newName = ui_->leJointName->text().toStdString();

	if (newName.empty() || newName == jointName || !poseItem->renameJoint(jointName, newName)) {
		ui_->leJointName->setText(QString::fromStdString(jointName));
	} else {
		jointName = newName;
	}
}

void PoseJointItem::onPositionEdited() {
    double newPos = ui_->leJointPos->text().toDouble();
	
    if (urdfModel) {
        auto pJoint = urdfModel->getJoint(jointName);
        if (pJoint) {
            newPos = max(pJoint->limits->lower, min(pJoint->limits->upper, newPos));
            ui_->leJointPos->setText(QString::number(newPos));
        }
    }

    poseItem->setJointPosition(jointName, newPos);
}

void PoseJointItem::onRemoveJoint() {
    poseItem->removeJoint(jointName);
}

PoseTreeItem::PoseTreeItem(QTreeWidget* parent, IScenarioInstance* _pScenario, string _poseName)
: QWidget(parent)
, pScenario(_pScenario)
, poseName(_poseName)
{
  SPose pose;
  const urdf::Model* urdf = pScenario->getURDFModel();

  QHBoxLayout* layout = new QHBoxLayout(this);
  lePoseName = new QLineEdit(QString::fromStdString(poseName), this);
  QLabel* robotLabel = new QLabel(this);
  layout->addWidget(lePoseName);
  layout->addWidget(robotLabel);
  this->setLayout(layout);
  layout->setMargin(0);

  containerItem = new QTreeWidgetItem(parent);
  containerItem->treeWidget()->setItemWidget(containerItem, 0, this);

  PoseAddJointButton* addBtn = new PoseAddJointButton(this, urdf);

  connect(lePoseName, SIGNAL(editingFinished()), this, SLOT(onNameEdited()));

  if (pScenario->getPose(poseName, pose)) {
    for (auto it = pose.jointPos.begin(); it != pose.jointPos.end(); it++) {
      PoseJointItem* jointItem = new PoseJointItem(this, it->first, it->second, urdf);
      jointItems[it->first] = jointItem;
    }
    robotLabel->setText(QString::fromStdString(pose.robotName));
  }

}

bool PoseTreeItem::renameJoint(string oldName, string newName) {
  return pScenario->renameJoint(poseName, oldName, newName);
}

void PoseTreeItem::setJointPosition(string joint, double pos) {
  pScenario->setJointPosition(poseName, joint, pos);
}

void PoseTreeItem::addJoint(string joint, double pos) {
  pScenario->addJoint(poseName, joint, pos);
}

void PoseTreeItem::removeJoint(string joint) {
  if (pScenario->removeJoint(poseName, joint) && jointItems.find(joint) != jointItems.end()) {
      containerItem->removeChild(jointItems[joint]->getItem());
  }
}

void PoseTreeItem::onNameEdited() {
    string newName = lePoseName->text().toStdString();
    if (newName != "" && newName != poseName) {
        AF result = pScenario->renamePose(poseName, newName);
        if (result) {
            poseName = newName;
        } else {
            cerr << result << endl;
        }
    }
    lePoseName->setText(QString::fromStdString(poseName));
}

void PoseTreeItem::setScenario(IScenarioInstance* _pScenario) {
  pScenario = _pScenario;
}

PosesWidget::PosesWidget(QWidget* parent)
  : QWidget( parent )
  , pScenario(0)
  , ui_(new Ui::PosesWidget())
{
  // set up the GUI
  ui_->setupUi(this);

  ui_->twPoses->setColumnCount(1);
}

PosesWidget::~PosesWidget() {
  delete ui_;
}

void PosesWidget::onPoseAdded(std::string pose) {
  PoseTreeItem* poseItem = new PoseTreeItem(ui_->twPoses, pScenario, pose);
  poseItems[pose] = poseItem;
}

void PosesWidget::onPoseRemoved(std::string pose) {
  auto it = poseItems.find(pose);
  if (it != poseItems.end()) {
    delete ui_->twPoses->takeTopLevelItem(ui_->twPoses->indexOfTopLevelItem(it->second->getItem()));
  }
}

void PosesWidget::onPosesCleared() {
  ui_->twPoses->clear();
  poseItems.clear();
}

void PosesWidget::onPosesLoaded() {
  for (auto it = pScenario->getContext()->poses.begin(); it != pScenario->getContext()->poses.end(); it++) {
    onPoseAdded(it->first);
  }
}

void PosesWidget::onPoseRenamed(std::string oldName, std::string newName) {
  
}

void PosesWidget::addEmptyPose() {
  pScenario->addEmptyPose();
}

void PosesWidget::setSelectedPose() {
  PoseTreeItem* treeItem = dynamic_cast<PoseTreeItem*>(ui_->twPoses->itemWidget(ui_->twPoses->currentItem(), 0));
  if (treeItem) {
    pScenario->setPose(treeItem->getPoseName());
  }
}

void PosesWidget::removeSelectedPose() {
  PoseTreeItem* treeItem = dynamic_cast<PoseTreeItem*>(ui_->twPoses->itemWidget(ui_->twPoses->currentItem(), 0));
  if (treeItem) {
    AF result = pScenario->removePose(treeItem->getPoseName());
    if (!result)
        cerr << result << endl;
    return;
  }
  cerr << "Selected item is no PoseTreeItem." << endl;
}

void PosesWidget::addCurrentPose() {
  pScenario->addCurrentPose();
}

void PosesWidget::filterURDF(bool bFilter) {
  if (bFilter) {
    string urdfName = pScenario->getURDFModel()->getName();
    for (auto it = poseItems.begin(); it != poseItems.end(); it++) {
      SPose pose;
      if (pScenario->getPose(it->first, pose)) {
        it->second->setEnabled(pose.robotName == urdfName);
      }
    }
  } else {
    for (auto it = poseItems.begin(); it != poseItems.end(); it++)
      it->second->setEnabled(true);
  }
}


void PosesWidget::setScenario(IScenarioInstance* _pScenario) {
  pScenario = _pScenario;
  pScenario->addPoseListener(this);
}

}
