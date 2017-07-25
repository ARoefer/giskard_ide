#include "giskard_sim/controller_widget.h"
#include "giskard_sim/utils.h"

#include "ui_controller_widget.h"

#include <QFileDialog>
#include <QErrorMessage>
#include <QFileSystemWatcher>

#include <giskard_core/giskard_core.hpp>

#include <unordered_set>

#include "giskard_sim/input_scalar_widget.h"
#include "giskard_sim/input_vector_widget.h"
#include "giskard_sim/input_rotation_widget.h"
#include "giskard_sim/input_frame_widget.h"

#include <rviz/frame_manager.h>

using namespace std;

namespace giskard_sim
{

ControllerWidget::ControllerWidget( QWidget* parent )
  : QWidget( parent )
  , pScenario(0)
  , ui_(new Ui::ControllerWidget())
  , frameManager(0)
{
  // set up the GUI
  ui_->setupUi(this);

  fileWatcher = new QFileSystemWatcher(this);

  connect(fileWatcher, SIGNAL(fileChanged(QString)), this, SLOT(onControllerFileChanged()));
}

ControllerWidget::~ControllerWidget() {
    delete ui_;
}

void ControllerWidget::setFrameManager(rviz::FrameManager* fm) {
    frameManager = fm;
    for (auto it = inputWidgets.begin(); it != inputWidgets.end(); it++) {
        if (dynamic_cast<InputVectorWidget*>(it->second.inputWidget)) {
            InputVectorWidget* ptr = dynamic_cast<InputVectorWidget*>(it->second.inputWidget);
            ptr->setFrameManager(frameManager);
        } else if (dynamic_cast<InputFrameWidget*>(it->second.inputWidget)) {
            InputFrameWidget* ptr = dynamic_cast<InputFrameWidget*>(it->second.inputWidget);
            ptr->setFrameManager(frameManager);
        } else if (dynamic_cast<InputRotationWidget*>(it->second.inputWidget)) {
            InputRotationWidget* ptr = dynamic_cast<InputRotationWidget*>(it->second.inputWidget);
            ptr->setFrameManager(frameManager);
        }
    }
}

void ControllerWidget::openLoadDialogue() {
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Controller"), "~/", tr("Giskard Controllers (*.yaml *.giskard)"));
    std::cout << "Selected file: " << fileName.toUtf8().constData() << std::endl;
    if (!fileName.isEmpty()) {
        string stdPath = fileName.toStdString();
        pScenario->changeControllerPath(stdPath);
    }
}

void ControllerWidget::openSaveDialogue() {

}

void ControllerWidget::onSaveController() {

}

void ControllerWidget::useRelativePath(bool bRelative) {
    AF result = pScenario->makeControllerRelative(bRelative);
    if (!result) {
       ui_->chkUsePackage->setChecked(pScenario->getContext()->controllerPath.packageRelative);
        QErrorMessage errorMessage(this);
       errorMessage.showMessage(QString::fromStdString(result.msg));
       return;
    }
    ui_->leControllerPath->setText(QString::fromStdString(pScenario->getContext()->controllerPath.path));
}

QString typeToString(const giskard_core::InputType& type) {
    switch(type) {
    case giskard_core::tScalar:
        return "scalar";
    case giskard_core::tVector3:
        return "vec3";
    case giskard_core::tRotation:
        return "rot";
    case giskard_core::tFrame:
        return "frame";
    case giskard_core::tJoint:
        return "joint";
    default:
        return "";
    }
}

void ControllerWidget::onControllerLoaded(giskard_core::QPController* controller) {
    string resPath = resolvePath(pScenario->getContext()->controllerPath);
    QString qPath = QString::fromStdString(resPath);
    if (fileWatcher->files().isEmpty()) {
      fileWatcher->addPath(qPath);
      controllerFileInfo = QFileInfo(qPath);
      lastModified = controllerFileInfo.lastModified();
    } else if (fileWatcher->files()[0] != qPath) {
      fileWatcher->removePaths(fileWatcher->files());
      fileWatcher->addPath(qPath);
      controllerFileInfo = QFileInfo(qPath);
      lastModified = controllerFileInfo.lastModified();
    }

    ui_->leControllerPath->setText(QString::fromStdString(pScenario->getContext()->controllerPath.path));
    ui_->chkUsePackage->setChecked(pScenario->getContext()->controllerPath.packageRelative);

    vector<string> jointList = controller->get_input_names(giskard_core::tJoint);
    unordered_set<string> jointSet(jointList.begin(), jointList.end());

    {
        auto it = jointItems.begin();
        while (it != jointItems.end()) {
            auto jIt = jointSet.find(it->first);
            if (jIt == jointSet.end()) {
                ui_->lwJoints->removeItemWidget(it->second);
                delete it->second;
                it = jointItems.erase(it);
            } else {
                jointSet.erase(jIt);
                it++;
            }
        }
    }

    for (auto jIt = jointSet.begin(); jIt != jointSet.end(); jIt++) {
        QListWidgetItem* pItem = new QListWidgetItem(QString::fromStdString(*jIt), ui_->lwJoints);
        jointItems[*jIt] = pItem;
    }

    ui_->lwJoints->sortItems();
}

void ControllerWidget::onInputAssignmentChanged(boost::shared_ptr<IInputAssignment> assignment) {
  auto it = inputWidgets.find(assignment->name);
  if (it != inputWidgets.end()) {
    switch (assignment->getType()) {
      case giskard_core::tScalar: {
        InputScalarWidget* widget = dynamic_cast<InputScalarWidget*>(it->second.inputWidget);
        if (!widget)
          throw std::invalid_argument("[ControllerWidget] Flying input type change is not supported. Delete the input first! Input: '"+assignment->name + "'");
          
        widget->setValue(boost::dynamic_pointer_cast<IScalarAssignment>(assignment));
      }
      break;
      case giskard_core::tVector3: {
        InputVectorWidget* widget = dynamic_cast<InputVectorWidget*>(it->second.inputWidget);
        if (!widget)
          throw std::invalid_argument("[ControllerWidget] Flying input type change is not supported. Delete the input first! Input: '"+assignment->name + "'");
          
        widget->setValue(boost::dynamic_pointer_cast<IVectorAssignment>(assignment));
      }
      break;
      case giskard_core::tRotation: {
        InputRotationWidget* widget = dynamic_cast<InputRotationWidget*>(it->second.inputWidget);
        if (!widget)
          throw std::invalid_argument("[ControllerWidget] Flying input type change is not supported. Delete the input first! Input: '"+assignment->name + "'");
          
        widget->setValue(boost::dynamic_pointer_cast<IRotationAssignment>(assignment));
      }
      break;
      case giskard_core::tFrame: {
        InputFrameWidget* widget = dynamic_cast<InputFrameWidget*>(it->second.inputWidget);
        if (!widget)
          throw std::invalid_argument("[ControllerWidget] Flying input type change is not supported. Delete the input first! Input: '"+assignment->name + "'");
          
        widget->setValue(boost::dynamic_pointer_cast<IFrameAssignment>(assignment));
      }
      break;
      default:
      break;
    }
  } else {
    if (assignment->getType() == giskard_core::tJoint)
      return;

    QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(assignment->name), ui_->inputList);
    ui_->inputList->addItem(item);
    switch (assignment->getType()) {
      case giskard_core::tScalar: {
        InputScalarWidget* widget = new InputScalarWidget(this, boost::dynamic_pointer_cast<IScalarAssignment>(assignment));
        inputWidgets[assignment->name] = {item, widget};
        ui_->inputList->setItemWidget(item, widget);
        connect(widget, SIGNAL(inputChanged(boost::shared_ptr<IInputAssignment>)), this, SLOT(assignmentChanged(boost::shared_ptr<IInputAssignment>)));
        item->setSizeHint(widget->sizeHint());
      }
      break;
      case giskard_core::tVector3: {
        InputVectorWidget* widget = new InputVectorWidget(this, boost::dynamic_pointer_cast<IVectorAssignment>(assignment), frameManager);
        inputWidgets[assignment->name] = {item, widget};
        ui_->inputList->setItemWidget(item, widget);
        connect(widget, SIGNAL(inputChanged(boost::shared_ptr<IInputAssignment>)), this, SLOT(assignmentChanged(boost::shared_ptr<IInputAssignment>)));
        item->setSizeHint(widget->sizeHint());
      }
      break;
      case giskard_core::tRotation: {
        InputRotationWidget* widget = new InputRotationWidget(this, boost::dynamic_pointer_cast<IRotationAssignment>(assignment), frameManager);
        inputWidgets[assignment->name] = {item, widget};
        ui_->inputList->setItemWidget(item, widget);
        connect(widget, SIGNAL(inputChanged(boost::shared_ptr<IInputAssignment>)), this, SLOT(assignmentChanged(boost::shared_ptr<IInputAssignment>)));
        item->setSizeHint(widget->sizeHint());
      }
      break;
      case giskard_core::tFrame: {
        InputFrameWidget* widget = new InputFrameWidget(this, boost::dynamic_pointer_cast<IFrameAssignment>(assignment), frameManager);
        inputWidgets[assignment->name] = {item, widget};
        ui_->inputList->setItemWidget(item, widget);
        connect(widget, SIGNAL(inputChanged(boost::shared_ptr<IInputAssignment>)), this, SLOT(assignmentChanged(boost::shared_ptr<IInputAssignment>)));
        item->setSizeHint(widget->sizeHint());
      }
      break;
      default:
      break;
    }
  }
}

void ControllerWidget::assignmentChanged(boost::shared_ptr<IInputAssignment> value) {
  value->setScenario(pScenario);
  pScenario->setInputAssignment(value);
}

void ControllerWidget::onInputAssignmentDeleted(const std::string& inputName) {
  auto it = inputWidgets.find(inputName);
  if (it != inputWidgets.end()) {
    ui_->inputList->removeItemWidget(it->second.item);
    delete it->second.inputWidget;
    delete it->second.item;
    inputWidgets.erase(it);
  }
}

void ControllerWidget::onInputsLoaded(const map<string, AssignmentPtr>& inputs) {
  for (auto it = inputs.begin(); it != inputs.end(); it++)
    onInputAssignmentChanged(it->second); 
}

void ControllerWidget::onInputsCleared() {
  for (auto it = inputWidgets.begin(); it != inputWidgets.end(); it++) {
    ui_->inputList->removeItemWidget(it->second.item);
    delete it->second.inputWidget;
    delete it->second.item;
  }
  inputWidgets.clear();
}

void ControllerWidget::onControllerLoadFailed(const std::string& msg) {
    string resPath = resolvePath(pScenario->getContext()->controllerPath);
    QString qPath = QString::fromStdString(resPath);
    if (fileWatcher->files().isEmpty()) {
      fileWatcher->addPath(qPath);
      controllerFileInfo = QFileInfo(qPath);
      lastModified = controllerFileInfo.lastModified();
    } else if (fileWatcher->files()[0] != qPath) {
      fileWatcher->removePaths(fileWatcher->files());
      fileWatcher->addPath(qPath);
      controllerFileInfo = QFileInfo(qPath);
      lastModified = controllerFileInfo.lastModified();
    }

    ui_->leControllerPath->setText(QString::fromStdString(pScenario->getContext()->controllerPath.path));
    ui_->chkUsePackage->setChecked(pScenario->getContext()->controllerPath.packageRelative);
}

void ControllerWidget::onControllerFileChanged() {
  controllerFileInfo.refresh();
  if (controllerFileInfo.lastModified() != lastModified) {
    lastModified = controllerFileInfo.lastModified();
    cout << "Controller File changed" << endl;
    pScenario->reloadController();
  }
}

void ControllerWidget::setScenario(IScenarioInstance* _pScenario) {
  pScenario = _pScenario;
  _pScenario->addControllerListener(this);
}

}
