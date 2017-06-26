#include "giskard_sim/object_info_widget.h"
#include "giskard_sim/utils.h"

#include <visualization_msgs/Marker.h>

#include "ui_object_info.h"

using namespace std;

namespace giskard_sim
{

ObjectInfoWidget::ObjectInfoWidget( QWidget* parent )
  : QWidget( parent )
  , ui_(new Ui::ObjectInfoWidget())
{
  // set up the GUI
  ui_->setupUi(this);

  ui_->vwPosition->showScaleSlider(false);
  ui_->vwRotation->showScaleSlider(false);
  ui_->cbUseMaterial->setEnabled(false);
  ui_->vwRotation->setLabelTexts(QString("R"),QString("P"),QString("Y"));
  ui_->vwScale->setVector(Eigen::Vector3d(1,1,1));

  ui_->cbVisual->addItem("Cube",     visualization_msgs::Marker::CUBE);
  ui_->cbVisual->addItem("Cylinder", visualization_msgs::Marker::CYLINDER);
  ui_->cbVisual->addItem("Mesh",     visualization_msgs::Marker::MESH_RESOURCE);
  ui_->cbVisual->addItem("Sphere",   visualization_msgs::Marker::SPHERE);
}

ObjectInfoWidget::~ObjectInfoWidget() {
    delete ui_;
}

void ObjectInfoWidget::onScaleChanged(Eigen::Vector3d scale) {

}

void ObjectInfoWidget::onPositionChanged(Eigen::Vector3d position) {

}

void ObjectInfoWidget::onRotationChanged(Eigen::Vector3d rotation) {

}

void ObjectInfoWidget::onColorChanged() {

}

void ObjectInfoWidget::onParentChanged(QString frame) {

}

void ObjectInfoWidget::onVisualChanged(int type) {

}

void ObjectInfoWidget::onNameChanged() {

}


}
