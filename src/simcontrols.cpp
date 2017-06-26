#include "giskard_sim/simcontrols.h"
#include "giskard_sim/datamodel.h"

#include "ui_simcontrols.h"

namespace giskard_sim
{

SimControls::SimControls( QWidget* parent )
  : QWidget( parent )
  , ui_(new Ui::SimControls())
{
  // set up the GUI
  ui_->setupUi(this);


  ui_->leSimTime->setValidator(new QDoubleValidator(0.0, 100.0, 8, this));

  // connecting the signals to the respectiv SLOTs
  /*
   * Main Control Buttons *
   */
  // the button is called pushButton_stop in the ui file
  //connect( ui_->pushButton_stop, SIGNAL( clicked() ), this, SLOT( gui_button_stop() ));
}

SimControls::~SimControls() {
  delete ui_;
}

  void SimControls::onScenarioLoaded(std::string path, const SScenarioContext* context) {
    const QString qPose = QString::fromStdString(context->simSettings.defaultPose);
    ui_->cboxStartPoses->setCurrentIndex(ui_->cboxStartPoses->findData(qPose));
    ui_->chkUseSimTime->setChecked(context->simSettings.bUseTimeStep);
    ui_->leSimTime->setText(QString::number(context->simSettings.timeStep));
    updatePlayPauseBtn(pScenario->getContext()->simSettings.bRunning);
  }

  void SimControls::onPoseAdded(std::string pose) {
      QString qPose = QString::fromStdString(pose);
      ui_->cboxStartPoses->addItem(qPose, qPose);
  }
  
  void SimControls::onPoseRemoved(std::string pose) {
      QString qPose = QString::fromStdString(pose);
      int idx = ui_->cboxStartPoses->findData(qPose);
      ui_->cboxStartPoses->removeItem(idx);
  }
  
  void SimControls::onPosesCleared() {
    ui_->cboxStartPoses->clear();
  }
  
  void SimControls::onPosesLoaded() {
    const SScenarioContext* pContext = pScenario->getContext();
    for (auto it = pContext->poses.begin(); it != pContext->poses.end(); it++) {
        QString qPose = QString::fromStdString(it->first);
        ui_->cboxStartPoses->addItem(qPose, qPose);
    }
  }

  void SimControls::onPoseRenamed(std::string oldName, std::string newName) {
      QString qOld = QString::fromStdString(oldName);
      QString qNew = QString::fromStdString(newName);
      int idx = ui_->cboxStartPoses->findData(qOld);
      ui_->cboxStartPoses->removeItem(idx);
      ui_->cboxStartPoses->insertItem(idx, qNew, qNew);
  }

  void SimControls::setTimeStep(QString str) {
    pScenario->setSimTimestep(str.toDouble());
    ui_->leSimTime->setText(QString::number(pScenario->getContext()->simSettings.timeStep));
  }
  
  void SimControls::timeStepEdited() {
    std::cout << "Signal editedTimeStep received!" << std::endl;
  }

  void SimControls::useTimeStep(bool bUse) {
    pScenario->setSimUseTimestep(bUse);
  }

  void SimControls::setStartPose(QString pose) {
    pScenario->setSimDefaultPose(pose.toStdString());
    QString qPose = QString::fromStdString(pScenario->getContext()->simSettings.defaultPose);
    ui_->cboxStartPoses->setCurrentIndex(ui_->cboxStartPoses->findData(qPose));
  }

  void SimControls::updatePlayPauseBtn(bool bPlay) {
      if (bPlay) {
          ui_->btnPlayPause->setText("Pause");
          ui_->btnPlayPause->setIcon(QIcon(":/icons/ic_pause_black_24dp.png"));
      } else {
          ui_->btnPlayPause->setText("Play");
          ui_->btnPlayPause->setIcon(QIcon(":/icons/ic_play_black_24dp.png"));
      }
  }

  void SimControls::toggleSim() {
    if (pScenario->setSimState(!pScenario->getContext()->simSettings.bRunning)) {
        updatePlayPauseBtn(pScenario->getContext()->simSettings.bRunning);
    }
  }

  void SimControls::resetSim() {
    pScenario->resetSim();
  }

  void SimControls::setScenario(IScenarioInstance* _pScenario) {
    pScenario = _pScenario;
    pScenario->addPoseListener(this);
    pScenario->addScenarioListener(this);
  }
}
