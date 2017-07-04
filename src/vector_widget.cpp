#include "giskard_sim/vector_widget.h"
#include "giskard_sim/utils.h"
#include "giskard_sim/decisive_double_validator.h"

#include "ui_vector_widget.h"

using namespace std;

namespace giskard_sim
{

VectorWidget::VectorWidget( QWidget* parent )
  : QWidget( parent )
  , ui_(new Ui::VectorWidget())
  , x(0)
  , y(0)
  , z(0)
  , scaleFactor(1)
  , bInScaleMode(false)
{
  // set up the GUI
  ui_->setupUi(this);

  DecisiveDoubleValidator* dVal = new DecisiveDoubleValidator(-10000.0, 10000.0, 6, this);
  ui_->leX->setValidator(dVal);
  ui_->leY->setValidator(dVal);
  ui_->leZ->setValidator(dVal);
}

VectorWidget::~VectorWidget() {
    delete ui_;
}

void VectorWidget::setLabelTexts(std::string lX, std::string lY, std::string lZ) {
  setLabelTexts(QString::fromStdString(lX), QString::fromStdString(lY), QString::fromStdString(lZ));
}

void VectorWidget::setVector(double _x, double _y, double _z) {
  x = _x;
  y = _y;
  z = _z;

  ui_->leX->setText(QString::number(x));
  ui_->leY->setText(QString::number(y));
  ui_->leZ->setText(QString::number(z));
}

void VectorWidget::setVector(geometry_msgs::Point vector) {
  setVector(vector.x, vector.y, vector.z);
}

void VectorWidget::setVector(geometry_msgs::Vector3 vector) {
  setVector(vector.x, vector.y, vector.z);
}

void VectorWidget::setVector(Eigen::Vector3d vector) {
  setVector(vector[0], vector[1], vector[2]);
}

void VectorWidget::setLabelTexts(QString lX, QString lY, QString lZ) {
  ui_->labelX->setText(lX);
  ui_->labelY->setText(lY);
  ui_->labelZ->setText(lZ);
}

void VectorWidget::showScaleSlider(bool bShow) {
  ui_->Scale->setVisible(bShow);
}

Eigen::Vector3d VectorWidget::getVectorEigen() {
  return Eigen::Vector3d(x,y,z);
}

geometry_msgs::Point VectorWidget::getRosPoint() {
  return rosPoint(x,y,z);
}

geometry_msgs::Vector3 VectorWidget::getRosVector() {
  return rosVec3(x,y,z);
}


void VectorWidget::onXChanged() {
  double tx = ui_->leX->text().toDouble();
  
  if (tx != x) {
    x = tx;
    Q_EMIT valueChanged(getVectorEigen());
  }
}

void VectorWidget::onYChanged() {
  double ty = ui_->leY->text().toDouble();
  if (ty != y) {
    y = ty;
    Q_EMIT valueChanged(getVectorEigen());
  }
}

void VectorWidget::onZChanged() {
  double tz = ui_->leZ->text().toDouble();
  if (tz != z) {
    z = tz;
    Q_EMIT valueChanged(getVectorEigen());
  }
}

void VectorWidget::onSliderMoved(int amount) {
  if (bInScaleMode) {
    scaleFactor = 0.1 * ui_->sliderScale->value();

    ui_->leX->setText(QString::number(x * scaleFactor));
    ui_->leY->setText(QString::number(y * scaleFactor));
    ui_->leZ->setText(QString::number(z * scaleFactor));
  }
}

void VectorWidget::onScaleBegin() {
  bInScaleMode = true;
}

void VectorWidget::onScaleEnd() {
  bInScaleMode = false;

  x *= scaleFactor;
  y *= scaleFactor;
  z *= scaleFactor;

  ui_->leX->setText(QString::number(x));
  ui_->leY->setText(QString::number(y));
  ui_->leZ->setText(QString::number(z));

  if (ui_->sliderScale->value() != 10) {
    Q_EMIT valueChanged(getVectorEigen());
  }

  ui_->sliderScale->setValue(10);
}


}
