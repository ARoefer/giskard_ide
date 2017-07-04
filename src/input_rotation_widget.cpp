#include "giskard_sim/input_rotation_widget.h"
#include "giskard_sim/utils.h"
#include "giskard_sim/decisive_double_validator.h"

#include "ui_input_rotation_widget.h"

#include <QLineEdit>

using namespace std;

namespace giskard_sim
{

InputRotationWidget::InputRotationWidget( QWidget* parent, ValuePtr _assignment)
  : QWidget( parent )
  , ui_(new Ui::InputRotationWidget())
  , assignment(_assignment)
{
  // set up the GUI
  ui_->setupUi(this);
  ui_->constRotation->showScaleSlider(false);
  ui_->labelName->setText(QString::fromStdString(assignment->name));

  connect(ui_->cbTargetFrame->lineEdit(), SIGNAL(editingFinished()), this, SLOT(dynamicFrameChanged()));
  connect(ui_->cbSourceFrame->lineEdit(), SIGNAL(editingFinished()), this, SLOT(dynamicRefFrameChanged()));

  setValue(assignment);
}

InputRotationWidget::~InputRotationWidget() {
    delete ui_;
}

void InputRotationWidget::setValue(ValuePtr value) {
  ui_->labelName->setText(QString::fromStdString(value->name));
  assignment = value;
  if (boost::dynamic_pointer_cast<ConstRotationAssignment>(assignment)) {
    boost::shared_ptr<ConstRotationAssignment> ptr = boost::dynamic_pointer_cast<ConstRotationAssignment>(assignment);
    ui_->dynamicWidget->setVisible(false);
    ui_->constRotation->setVisible(true);

    ui_->constRotation->setVector(toEulerDeg(ptr->getValue()));
    ui_->cbIsConst->setChecked(true);
  } else if (boost::dynamic_pointer_cast<RotationAssignment>(assignment)){
    boost::shared_ptr<RotationAssignment> ptr = boost::dynamic_pointer_cast<RotationAssignment>(assignment);    
    ui_->dynamicWidget->setVisible(true);
    ui_->constRotation->setVisible(false);

    ui_->cbSourceFrame->lineEdit()->setText(QString::fromStdString(ptr->target));
    ui_->cbTargetFrame->lineEdit()->setText(QString::fromStdString(ptr->object));
    ui_->cbIsConst->setChecked(false);
  }
}

void InputRotationWidget::constantRotationChanged(Eigen::Vector3d newVal) {
  Eigen::Quaterniond newRot = fromEulerDeg(newVal);
  boost::shared_ptr<ConstRotationAssignment> cPtr = boost::dynamic_pointer_cast<ConstRotationAssignment>(assignment);
  if (cPtr && cPtr->getValue() != newRot) {
    assignment = ValuePtr(new ConstRotationAssignment(cPtr->name, newRot));
    Q_EMIT inputChanged(assignment);
  }
}

void InputRotationWidget::dynamicFrameChanged() {
    dynamicFrameChanged(ui_->cbTargetFrame->lineEdit()->text());
}

void InputRotationWidget::dynamicRefFrameChanged() {
    dynamicRefFrameChanged(ui_->cbSourceFrame->lineEdit()->text());
}

void InputRotationWidget::dynamicFrameChanged(QString newTarget) {
    std::string newValue = newTarget.toStdString();
    cout << "New target frame: " << newValue << endl;
    boost::shared_ptr<RotationAssignment> ptr = boost::dynamic_pointer_cast<RotationAssignment>(assignment);
    if (ptr && newValue != ptr->object) {
        assignment = ValuePtr(new RotationAssignment(assignment->name, newValue, ptr->target));
        Q_EMIT inputChanged(assignment);
    }
}

void InputRotationWidget::dynamicRefFrameChanged(QString newSource) {
    std::string newValue = newSource.toStdString();
    cout << "New source frame: " << newValue << endl;
    boost::shared_ptr<RotationAssignment> ptr = boost::dynamic_pointer_cast<RotationAssignment>(assignment);
    if (ptr && newValue != ptr->target) {
        assignment = ValuePtr(new RotationAssignment(assignment->name, ptr->object, newValue));
        Q_EMIT inputChanged(assignment);
    }
}

void InputRotationWidget::setConstant(bool bConst) {
  if (bConst) {
    ui_->dynamicWidget->setVisible(false);
    ui_->constRotation->setVisible(true);
  
    assignment = ValuePtr(new ConstRotationAssignment(assignment->name, fromEulerDeg(ui_->constRotation->getVectorEigen())));
    Q_EMIT inputChanged(assignment);
  } else {
    ui_->dynamicWidget->setVisible(true);
    ui_->constRotation->setVisible(false);
    
    // TODO: Parse ui input
    assignment = ValuePtr(new RotationAssignment(assignment->name, ui_->cbTargetFrame->lineEdit()->text().toStdString(), ui_->cbSourceFrame->lineEdit()->text().toStdString()));
    Q_EMIT inputChanged(assignment);
  }
}


}
