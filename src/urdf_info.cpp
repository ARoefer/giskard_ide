#include "giskard_sim/urdf_info.h"
#include "giskard_sim/utils.h"


#include "ui_urdf.h"

#include <QFileDialog>
#include <QErrorMessage>

#include <urdf/model.h>

namespace giskard_sim
{

URDFInfo::URDFInfo( QWidget* parent )
  : QWidget( parent )
  , pScenario(0)
  , ui_(new Ui::URDFInfo())
{
  // set up the GUI
  ui_->setupUi(this);
  ui_->labelRobotName->setText("No URDF loaded");
}

URDFInfo::~URDFInfo() {
  delete ui_;
}

void URDFInfo::openLoadDialogue() {
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Scenario"), "~/", tr("*.urdf"));
    std::cout << "Selected file: " << fileName.toUtf8().constData() << std::endl;
    if (!fileName.isEmpty()) {
        string stdPath = fileName.toStdString();
        pScenario->changeURDFPath(stdPath);
    }
}

void URDFInfo::useRelativePath(bool bRelative) {
    AF result = pScenario->makeURDFRelative(bRelative);
    if (!result) {
       ui_->checkBox->setChecked(pScenario->getContext()->urdfPath.packageRelative);
        QErrorMessage errorMessage(this);
       errorMessage.showMessage(QString::fromStdString(result.msg));
       return;
    }
    ui_->leUrdfPath->setText(QString::fromStdString(pScenario->getContext()->urdfPath.path));
}

void URDFInfo::onURDFChanged(const urdf::Model* model) {
    if (model) {
        ui_->labelRobotName->setText(QString::fromStdString(model->getName()));
    } else {
        ui_->labelRobotName->setText(QString("LOAD FAILED!"));
    }
    ui_->leUrdfPath->setText(QString::fromStdString(pScenario->getContext()->urdfPath.path));
    ui_->checkBox->setChecked(pScenario->getContext()->urdfPath.packageRelative);
}

void URDFInfo::setScenario(IScenarioInstance* _pScenario) {
  pScenario = _pScenario;
  _pScenario->addURDFListener(this);
}

}
