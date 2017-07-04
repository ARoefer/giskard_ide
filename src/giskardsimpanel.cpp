#include "giskard_sim/giskardsimpanel.h"

#include "ui_giskardsimpanel.h"

#include <rviz/visualization_manager.h>
#include <rviz/display_group.h>
#include <rviz/display.h>
#include <rviz/default_plugin/robot_model_display.h>
#include <rviz/default_plugin/interactive_marker_display.h>
#include <rviz/default_plugin/marker_array_display.h>

#include "giskard_sim/robot_model_display.h"

#include <QDir>

#include <pwd.h>


namespace giskard_sim
{

GiskardSimPanel::GiskardSimPanel( QWidget* parent )
  : rviz::Panel( parent )
  , modelDisplay(0)
  , intDisplay(0)
  , ui_(new Ui::GiskardSimPanel())
{
  // set up the GUI
  ui_->setupUi(this);

  // connecting the signals to the respectiv SLOTs
  /*
   * Main Control Buttons *
   */
  // the button is called pushButton_stop in the ui file
  //connect( ui_->pushButton_stop, SIGNAL( clicked() ), this, SLOT( gui_button_stop() ));

  scenario.addErrorListener(this);

  ui_->scenarioInfoWidget->setScenario(&scenario);
  ui_->urdfWidget->setScenario(&scenario);
  ui_->posesWidget->setScenario(&scenario);
  ui_->controllerWidget->setScenario(&scenario);
  ui_->controlWidget->setScenario(&scenario);
  ui_->sceneWidget->setScenario(&scenario);
}

GiskardSimPanel::~GiskardSimPanel() {
//    if (modelDisplay)
//        delete modelDisplay;
//    if (intDisplay)
//        delete intDisplay;

    delete ui_;
}

void GiskardSimPanel::load( const rviz::Config& config ) {
    rviz::Panel::load(config);

    QString scenarioPath;
    if (config.mapGetString("giskard_scenario_path", &scenarioPath)) {
        scenario.loadFromYAML(scenarioPath.toStdString());
    }
}

void GiskardSimPanel::save( rviz::Config config ) const {
    rviz::Panel::save( config );
    string path = "";//ui_->scenarioInfoWidget->getPath();
    if (path.empty()) {
        path = QDir::homePath().toStdString();
        path += "/.rviz/last_giskard_scenario.yaml";
    }

    config.mapSetValue( "giskard_scenario_path", QString::fromStdString(path));

    YAML::Node node = YAML::Load("");
    const SScenarioContext* psc = scenario.getContext();
    node = (*(psc));
    std::ofstream fout(path);
    fout << node;
    fout.close();
}

void GiskardSimPanel::onInitialize() {
    rviz::DisplayGroup* pDG = vis_manager_->getRootDisplayGroup();

    QString modelName = "Giskard Sim: Robot Model - DON'T TOUCH";
    QString intName = "Giskard Sim: Interactive Markers - DON'T TOUCH";

    for (int i = 0; i < pDG->numDisplays(); i++) {
        if (pDG->getDisplayAt(i)->getName() == modelName) {
            modelDisplay = dynamic_cast<RobotModelDisplay*>(pDG->getDisplayAt(i));
        } else if (pDG->getDisplayAt(i)->getName() == intName) {
            intDisplay = dynamic_cast<rviz::InteractiveMarkerDisplay*>(pDG->getDisplayAt(i));
        }
    }

    if (!modelDisplay) {
        modelDisplay = dynamic_cast<RobotModelDisplay*>(vis_manager_->createDisplay("giskard_sim/RobotModel", modelName, true));
    }
    scenario.addURDFListener(modelDisplay);

    if (!intDisplay) {
        intDisplay = dynamic_cast<rviz::InteractiveMarkerDisplay*>(vis_manager_->createDisplay("rviz/InteractiveMarkers", intName, true));
        //intDisplay->setMar(QString("/giskard_marker_server/update"));
    }
}

void GiskardSimPanel::onLoadScenarioFailed(const std::string& msg) {
  ui_->tabWidget->setCurrentWidget(ui_->teLog);
  ui_->teLog->append(QString::fromStdString(msg + "\n"));
}

void GiskardSimPanel::onLoadURDFFailed(const std::string& msg) {
  ui_->tabWidget->setCurrentWidget(ui_->teLog);
    ui_->teLog->append(QString::fromStdString(msg + "\n"));
}
void GiskardSimPanel::onLoadControllerFailed(const std::string& msg) {
  ui_->tabWidget->setCurrentWidget(ui_->teLog);
    ui_->teLog->append(QString::fromStdString(msg + "\n"));
}

void GiskardSimPanel::onRunControllerFailed(const std::string& msg) {
  ui_->tabWidget->setCurrentWidget(ui_->teLog);
    ui_->teLog->append(QString::fromStdString(msg + "\n"));
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(giskard_sim::GiskardSimPanel, rviz::Panel)

PLUGINLIB_EXPORT_CLASS(giskard_sim::RobotModelDisplay, rviz::Display)
