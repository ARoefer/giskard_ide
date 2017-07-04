#include "giskard_sim/input_scalar_widget.h"
#include "giskard_sim/utils.h"
#include "giskard_sim/decisive_double_validator.h"

#include "ui_input_scalar_widget.h"

#include <QLineEdit>

using namespace std;

namespace giskard_sim
{

InputScalarWidget::InputScalarWidget( QWidget* parent, ValuePtr _assignment)
  : QWidget( parent )
  , ui_(new Ui::InputScalarWidget())
  , assignment(_assignment)
{
  // set up the GUI
  ui_->setupUi(this);
  ui_->leScalar->setValidator(new DecisiveDoubleValidator(-10000.0, 10000.0, 6, this));
  ui_->labelName->setText(QString::fromStdString(assignment->name));
  ui_->cbIsConst->setVisible(false);
  setValue(assignment);
}

InputScalarWidget::~InputScalarWidget() {
    delete ui_;
}

void InputScalarWidget::setValue(ValuePtr value) {
  ui_->labelName->setText(QString::fromStdString(value->name));
  assignment = value;
  if (boost::dynamic_pointer_cast<ConstScalarAssignment>(assignment)) {
    boost::shared_ptr<ConstScalarAssignment> ptr = boost::dynamic_pointer_cast<ConstScalarAssignment>(assignment);
    ui_->cbDynamicScalar->setVisible(false);
    ui_->leScalar->setVisible(true);

    ui_->leScalar->setText(QString::number(ptr->getValue()));
    ui_->cbIsConst->setChecked(true);
  } 
  // else if (boost::dynamic_pointer_cast<VectorPositionAssignment>(assignment)){
  //   boost::shared_ptr<VectorPositionAssignment> ptr = boost::dynamic_pointer_cast<VectorPositionAssignment>(assignment);    
  //   ui_->dynamicWidget->setVisible(true);
  //   ui_->constVector->setVisible(false);

  //   ui_->cbSourceFrame->setText(QString::fromStdString(ptr->object));
  //   ui_->cbTargetFrame->setText(QString::fromStdString(ptr->target));
  //   ui_->cbIsConst->setChecked(false);
  // }
}

void InputScalarWidget::constantScalarEdited() {
  double newVal = ui_->leScalar->text().toDouble();

  boost::shared_ptr<ConstScalarAssignment> cPtr = boost::dynamic_pointer_cast<ConstScalarAssignment>(assignment);
  if (cPtr && cPtr->getValue() != newVal) {
    assignment = ValuePtr(new ConstScalarAssignment(cPtr->name, newVal));
    Q_EMIT inputChanged(assignment);
  }
}

void InputScalarWidget::dynamicScalarChanged(QString newVec) {
  // TODO: Dynamic vector
}

void InputScalarWidget::setConstant(bool bConst) {
  if (bConst) {
    ui_->cbDynamicScalar->setVisible(false);
    ui_->leScalar->setVisible(true);
  
    assignment = ValuePtr(new ConstScalarAssignment(assignment->name, ui_->leScalar->text().toDouble()));
    Q_EMIT inputChanged(assignment);
  } else {
    ui_->cbDynamicScalar->setVisible(true);
    ui_->leScalar->setVisible(false);
    
    // TODO: Parse ui input
  }
}


}
