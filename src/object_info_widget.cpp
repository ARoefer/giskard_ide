#include "giskard_sim/object_info_widget.h"
#include "giskard_sim/utils.h"

#include <QColorDialog>

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
  ui_->meshPath->setEnabled(false);
  ui_->cbUseMaterial->setEnabled(false);
  ui_->vwRotation->setLabelTexts(QString("R"),QString("P"),QString("Y"));
  ui_->vwScale->setVector(Eigen::Vector3d(1,1,1));

  ui_->cbVisual->addItem("Cube",     visualization_msgs::Marker::CUBE);
  ui_->cbVisual->addItem("Cylinder", visualization_msgs::Marker::CYLINDER);
  ui_->cbVisual->addItem("Mesh",     visualization_msgs::Marker::MESH_RESOURCE);
  ui_->cbVisual->addItem("Sphere",   visualization_msgs::Marker::SPHERE);

  connect(ui_->cbParentFrame->lineEdit(), SIGNAL(editingFinished()), this, SLOT(onParentMaybeChanged()));
  ui_->meshPath->addSuffix(".dae");

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
  int idx = ui_->cbVisual->findData(obj.visual.type);
  if (idx != -1) {
    ui_->cbVisual->setCurrentIndex(idx);
  }
  if (obj.visual.type == visualization_msgs::Marker::MESH_RESOURCE) {
      ui_->meshPath->setEnabled(true);
      ui_->cbUseMaterial->setEnabled(true);
      ui_->cbUseMaterial->setChecked(obj.visual.mesh_use_embedded_materials);
  } else {
      ui_->meshPath->setEnabled(false);
      ui_->cbUseMaterial->setEnabled(false);
  }

  Eigen::Vector3d rpy = toEulerDeg(object.transform.rotation());

  ui_->vwRotation->setVector(rpy);
  ui_->btnColor->setStyleSheet("background-color: "+ QString::fromStdString(toRGBACSS(object.visual.color)));
}

void ObjectInfoWidget::onMeshChanged(std::string path) {
    if (object.visual.mesh_resource != path) {
        object.visual.mesh_resource = path;

        Q_EMIT objectEdited(object);
    }
}

void ObjectInfoWidget::setFrameManager(rviz::FrameManager* frameManager) {
    ui_->cbParentFrame->setFrameManager(frameManager);
}

void ObjectInfoWidget::onScaleChanged(Eigen::Vector3d scale) {
  object.visual.scale = rosVec3(scale);

  Q_EMIT objectEdited(object);
}

void ObjectInfoWidget::onPositionChanged(Eigen::Vector3d position) {
  object.transform = Eigen::Translation3d(position) * object.transform.rotation();

  Q_EMIT objectEdited(object);
}

void ObjectInfoWidget::onRotationChanged(Eigen::Vector3d rotation) {
  Eigen::Affine3d rot(fromEulerDeg(rotation));
  object.transform = ((Eigen::Translation3d) object.transform.translation()) * rot;

  Q_EMIT objectEdited(object);
}

void ObjectInfoWidget::onColorChanged() {
  QColor color = QColorDialog::getColor(toQColor(object.visual.color));
  std_msgs::ColorRGBA newColor = rosColorRGBA(color);
  std_msgs::ColorRGBA oldColor = object.visual.color;
  if (oldColor.r != newColor.r || oldColor.g != newColor.g || oldColor.b != newColor.b || oldColor.a != newColor.a) {
      object.visual.color = newColor;
      Q_EMIT objectEdited(object);
  }
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

        Q_EMIT objectEdited(object);
    }
}

void ObjectInfoWidget::onVisualChanged(int type) {
  int newVal = ui_->cbVisual->itemData(type).toInt();
  if (object.visual.type != newVal) {
    object.visual.type = newVal;
    Q_EMIT objectEdited(object);
  }
}

void ObjectInfoWidget::onNameChanged() {
  object.name = ui_->leName->text().toStdString();

  Q_EMIT objectEdited(object);
}

void ObjectInfoWidget::onUseMaterialsChanged(int state) {
  bool use = state == Qt::Checked;

  if (use != object.visual.mesh_use_embedded_materials) {
    object.visual.mesh_use_embedded_materials = use;

    Q_EMIT objectEdited(object);
  }
}

SWorldObject ObjectInfoWidget::getObject() const {
  return object;
}

}
