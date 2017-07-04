#include "giskard_sim/scene_widget.h"
#include "giskard_sim/utils.h"
#include "giskard_sim/create_object_dialog.h"

#include "ui_scene_widget.h"

using namespace std;

namespace giskard_sim
{

SceneWidget::SceneWidget( QWidget* parent )
  : QWidget( parent )
  , pScenario(0)
  , ui_(new Ui::SceneWidget())
{
  // set up the GUI
  ui_->setupUi(this);
  ui_->objectInfoWidget->setEnabled(false);
}

SceneWidget::~SceneWidget() {
    delete ui_;
}

void SceneWidget::createObject() {
  CreateObjectDialog dialog(this);
  connect(&dialog, SIGNAL(createObject(SWorldObject)), this, SLOT(spawnObject(SWorldObject)));
  dialog.exec();
}

void SceneWidget::deleteObject() {

}

void SceneWidget::selectedObjectChanged() {

}

void SceneWidget::spawnObject(SWorldObject object) {
    cout << "Spawn object called" << endl;
  pScenario->addSceneObject(&object);
}

void SceneWidget::setScenario(IScenarioInstance* _pScenario) {
  pScenario = _pScenario;
}

}
