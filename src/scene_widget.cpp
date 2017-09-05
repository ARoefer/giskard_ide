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
  , frameManager(0)
{
  // set up the GUI
  ui_->setupUi(this);
  ui_->objectInfoWidget->setEnabled(false);
  ui_->listWidget->setSortingEnabled(true);
}

SceneWidget::~SceneWidget() {
    delete ui_;
}

void SceneWidget::createObject() {
  CreateObjectDialog dialog(this, frameManager);
  connect(&dialog, SIGNAL(createObject(SWorldObject)), this, SLOT(spawnObject(SWorldObject)));
  dialog.exec();
}

void SceneWidget::setFrameManager(rviz::FrameManager* fM) {
    frameManager = fM;
    ui_->objectInfoWidget->setFrameManager(frameManager);
}

void SceneWidget::deleteObject() {
    QListWidgetItem* pItem = ui_->listWidget->currentItem();
    if (pItem)
      pScenario->removeSceneObject(pItem->text().toStdString());
}

void SceneWidget::selectedObjectChanged() {
    QListWidgetItem* pItem = ui_->listWidget->currentItem();
    if (pItem)
      pScenario->selectSceneObject(pItem->text().toStdString());
}

void SceneWidget::spawnObject(SWorldObject object) {
    cout << "Spawn object called" << endl;
    pScenario->addSceneObject(&object);
}

void SceneWidget::onScenarioLoaded(std::string path, const SScenarioContext* context) {
  for (auto it = context->objects.begin(); it != context->objects.end(); it++) {
    onObjectAdded(*(it->second));
  }
}

void SceneWidget::onObjectAdded(const SWorldObject& object) {
  auto list = ui_->listWidget->findItems(QString::fromStdString(object.name), Qt::MatchExactly);
  if (list.empty())
    ui_->listWidget->addItem(QString::fromStdString(object.name));
}

void SceneWidget::onObjectChanged(const SWorldObject& object) {
  if (selected == object.name) {
    ui_->objectInfoWidget->setObject(object);
  }
}

void SceneWidget::onObjectRemoved(const std::string& name) {
  if (name == selected) {
    ui_->objectInfoWidget->setEnabled(false);
    ui_->listWidget->takeItem(ui_->listWidget->currentRow());
  }
}

void SceneWidget::onSelectedObjectChanged(const std::string& selected) {
  if (selected.empty()) {
    ui_->objectInfoWidget->setEnabled(false);
  } else if (this->selected != selected) {
    this->selected = selected;
    ui_->objectInfoWidget->setEnabled(true);
    ui_->objectInfoWidget->setObject(*(pScenario->getContext()->objects.find(selected)->second));
    auto list = ui_->listWidget->findItems(QString::fromStdString(selected), Qt::MatchExactly);
    if (!list.empty())
      ui_->listWidget->setCurrentItem(list[0]);
  }
}

void SceneWidget::objectChanged(SWorldObject object) {
  if (object.name == selected) {
    pScenario->addSceneObject(&object);
  } else {
    pScenario->removeSceneObject(selected);
    pScenario->addSceneObject(&object);
    pScenario->selectSceneObject(object.name);
  }
}

void SceneWidget::onObjectsCleared() {
  ui_->listWidget->clear();
  ui_->objectInfoWidget->setEnabled(false);
  selected = "";
}

void SceneWidget::setScenario(IScenarioInstance* _pScenario) {
  pScenario = _pScenario;
  pScenario->addScenarioListener(this);
}

}
