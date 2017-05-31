#include "giskard_sim/controller_widget.h"
#include "giskard_sim/utils.h"

#include "ui_controller_widget.h"

#include <QFileDialog>
#include <QErrorMessage>
#include <QFileSystemWatcher>

#include <giskard/giskard.hpp>

#include <unordered_set>

using namespace std;

namespace giskard_sim
{

ControllerWidget::ControllerWidget( QWidget* parent )
  : QWidget( parent )
  , pScenario(0)
  , ui_(new Ui::ControllerWidget())
{
  // set up the GUI
  ui_->setupUi(this);

  fileWatcher = new QFileSystemWatcher(this);

  connect(fileWatcher, SIGNAL(fileChanged(QString)), this, SLOT(onControllerFileChanged()));
}

ControllerWidget::~ControllerWidget() {
    delete ui_;
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

QString typeToString(giskard::Scope::InputTypes& type) {
    switch(type) {
    case giskard::Scope::Scalar:
        return "scalar";
    case giskard::Scope::Vector:
        return "vec3";
    case giskard::Scope::Rotation:
        return "rot";
    case giskard::Scope::Frame:
        return "frame";
    case giskard::Scope::Joint:
        return "joint";
    default:
        return "";
    }
}

void ControllerWidget::onControllerLoaded(giskard::QPController* controller) {
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

    vector<string> jointList = controller->get_scope().get_joint_inputs();
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

    map<const string, giskard::Scope::ScopeInput> inputs = controller->get_scope().get_inputs();
    int row = 0;
    ui_->twInputs->setRowCount(inputs.size() - jointList.size());
    for (auto it = inputs.begin(); it != inputs.end(); it++) {
        if (it->second.type == giskard::Scope::Joint)
            continue;

        auto iit = inputWidgets.find(it->first);
        if (iit != inputWidgets.end()) {
            iit->second.type->setText(typeToString(it->second.type));
            ui_->twInputs->setItem(row, 0, iit->second.name);
            ui_->twInputs->setItem(row, 1, iit->second.type);
            ui_->twInputs->setItem(row, 2, iit->second.value);
            iit->second.row = row;
        } else {
            STableRow newRow;
            newRow.name = new QTableWidgetItem(QString::fromStdString(it->first));
            newRow.type = new QTableWidgetItem(typeToString(it->second.type));
            newRow.value = new QTableWidgetItem("lel");
            newRow.row = row;

            ui_->twInputs->setItem(row, 0, newRow.name);
            ui_->twInputs->setItem(row, 1, newRow.type);
            ui_->twInputs->setItem(row, 2, newRow.value);
            inputWidgets[it->first] = newRow;
        }
        row++;
    }

    auto it = inputWidgets.begin();
    while (it != inputWidgets.end()) {
        if (inputs.find(it->first) == inputs.end()) {
//            delete it->second.name;
//            delete it->second.type;
//            delete it->second.value;
            it = inputWidgets.erase(it);
        } else
            it++;
    }
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
