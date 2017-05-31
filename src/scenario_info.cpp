#include "giskard_sim/scenario_info.h"

#include "ui_scenario_info.h"
#include "giskard_sim/datamodel.h"

#include <QFileDialog>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace giskard_sim
{

ScenarioInfo::ScenarioInfo( QWidget* parent )
  : QWidget( parent )
  , ui_(new Ui::ScenarioInfo())
  , pScenario(0)
{
  // set up the GUI
  ui_->setupUi(this);
}

ScenarioInfo::~ScenarioInfo() {
  delete ui_;
}

  void ScenarioInfo::setScenario(IScenarioInstance* _pScenario) {
      pScenario = _pScenario;
      pScenario->addScenarioListener(this);
  }

  void ScenarioInfo::openSaveDialogue() {
      std::string startingFile = "~/"+ pScenario->getContext()->name +".yaml";
      QString currentFile = ui_->lePath->text();
      if (!currentFile.isEmpty()) {
          startingFile = currentFile.left(currentFile.lastIndexOf('/') + 1).toStdString() + pScenario->getContext()->name +".yaml";
      }

      QString fileName = QFileDialog::getSaveFileName(this, tr("Save Scenario"), startingFile.c_str(), tr("*.yaml"));
      std::cout << "Selected file: " << fileName.toUtf8().constData() << std::endl;
      if (!fileName.isEmpty()) {
            YAML::Node node = YAML::Load("");
            node = (*(pScenario->getContext()));
            std::ofstream fout(fileName.toStdString());
            fout << node;
            fout.close();
            ui_->lePath->setText(fileName);
      }
  }

  void ScenarioInfo::openLoadDialogue() {
      QString fileName = QFileDialog::getOpenFileName(this, tr("Open Scenario"), "~/", tr("*.yaml"));
      std::cout << "Selected file: " << fileName.toUtf8().constData() << std::endl;
      if (!fileName.isEmpty()) {
        pScenario->loadFromYAML(fileName.toStdString());
      }
  }

  void ScenarioInfo::saveScenario() {
    if (ui_->lePath->text().isEmpty()) {
        openSaveDialogue();
    } else {
        YAML::Node node = YAML::Load("");
        node = (*(pScenario->getContext()));
        std::ofstream fout(ui_->lePath->text().toStdString());
        fout << node;
        fout.close();
    }
  }
  
  void ScenarioInfo::scenarioNameEdited() {
    if (!ui_->leName->text().isEmpty()) {
        pScenario->renameScenario(ui_->leName->text().toStdString());
    } else {
        ui_->leName->setText(QString::fromStdString(pScenario->getContext()->name));
    }
  }

  void ScenarioInfo::onScenarioLoaded(string path, const SScenarioContext* context) {
    ui_->lePath->setText(QString::fromStdString(path));
    ui_->leName->setText(QString::fromStdString(context->name));
  }

  std::string ScenarioInfo::getPath() {
      return ui_->lePath->text().toStdString();
  }
}
