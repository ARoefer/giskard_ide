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

  connect(ui_->cbParentFrame->lineEdit(), SIGNAL(editingFinished()), this, SLOT(onParentMaybeChanged()));
}

ObjectInfoWidget::~ObjectInfoWidget() {
    delete ui_;
}

void ObjectInfoWidget::setObject(SWorldObject obj) {
  object = obj;
  ui_->leName->setText(QString::fromStdString(obj.name));
  ui_->cbParentFrame->setEditText(QString::fromStdString(obj.parent));
  ui_->vwPosition->setVector(object.transform.translation());
  ui_->vwScale->setVector(object.visual.scale);
  Eigen::Vector3d rpy = toEulerDeg(object.transform.rotation());

  ui_->vwRotation->setVector(rpy);
  ui_->leColor->setColor(toQColor(object.visual.color));
}

void ObjectInfoWidget::onScaleChanged(Eigen::Vector3d scale) {
  object.visual.scale = rosVec3(scale);

  // TODO: Update
}

void ObjectInfoWidget::onPositionChanged(Eigen::Vector3d position) {
  object.transform = Eigen::Translation3d(position) * object.transform.rotation();

  // TODO: Update
}

void ObjectInfoWidget::onRotationChanged(Eigen::Vector3d rotation) {
  Eigen::Affine3d rot(fromEulerDeg(rotation));
  object.transform = ((Eigen::Translation3d) object.transform.translation()) * rot;

  // TODO: Update
}

void ObjectInfoWidget::onColorChanged() {
  object.visual.color = rosColorRGBA(ui_->leColor->getColor());

  // TODO: Update
}

void ObjectInfoWidget::onParentMaybeChanged() {
    QString newParent = ui_->cbParentFrame->lineEdit()->text();
    if (newParent != QString::fromStdString(object.parent))
        onParentChanged(newParent);
}

void ObjectInfoWidget::onParentChanged(QString frame) {
    std::string newParent = frame.toStdString();
    if (newParent != object.parent) {
        object.parent = newParent;

        // TODO: Update
    }
}

void ObjectInfoWidget::onVisualChanged(int type) {
  object.visual.type = ui_->cbVisual->itemData(type).toInt();

  // TODO: Update
}

void ObjectInfoWidget::onNameChanged() {
  object.name = ui_->leName->text().toStdString();

  // TODO: Update
}

SWorldObject ObjectInfoWidget::getObject() const {
  return object;
}

}
