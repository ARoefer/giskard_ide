#include "giskard_sim/input_vector_widget.h"
#include "giskard_sim/utils.h"

#include "ui_input_vector_widget.h"

#include <QLineEdit>

using namespace std;

namespace giskard_sim
{

InputVectorWidget::InputVectorWidget( QWidget* parent, ValuePtr _assignment)
  : QWidget( parent )
  , ui_(new Ui::InputVectorWidget())
  , assignment(_assignment)
{
  // set up the GUI
  ui_->setupUi(this);
  setValue(assignment);

  connect(ui_->cbTargetFrame->lineEdit(), SIGNAL(editingFinished()), this, SLOT(dynamicFrameChanged()));
  connect(ui_->cbSourceFrame->lineEdit(), SIGNAL(editingFinished()), this, SLOT(dynamicRefFrameChanged()));
}

InputVectorWidget::~InputVectorWidget() {
    delete ui_;
}

void InputVectorWidget::setValue(ValuePtr value) {
  ui_->labelName->setText(QString::fromStdString(value->name));
  assignment = value;
  if (boost::dynamic_pointer_cast<ConstVectorAssignment>(assignment)) {
    boost::shared_ptr<ConstVectorAssignment> ptr = boost::dynamic_pointer_cast<ConstVectorAssignment>(assignment);
    ui_->dynamicWidget->setVisible(false);
    ui_->constVector->setVisible(true);

    ui_->constVector->setVector(ptr->getValue());
    ui_->cbIsConst->setChecked(true);
  } else if (boost::dynamic_pointer_cast<VectorPositionAssignment>(assignment)){
    boost::shared_ptr<VectorPositionAssignment> ptr = boost::dynamic_pointer_cast<VectorPositionAssignment>(assignment);    
    ui_->dynamicWidget->setVisible(true);
    ui_->constVector->setVisible(false);

    ui_->cbSourceFrame->lineEdit()->setText(QString::fromStdString(ptr->target));
    ui_->cbTargetFrame->lineEdit()->setText(QString::fromStdString(ptr->object));
    ui_->cbIsConst->setChecked(false);
  }
}

void InputVectorWidget::constantVectorChanged(Eigen::Vector3d vector) {
  boost::shared_ptr<ConstVectorAssignment> cPtr = boost::dynamic_pointer_cast<ConstVectorAssignment>(assignment);
  if (cPtr && cPtr->getValue() != vector) {
    assignment = ValuePtr(new ConstVectorAssignment(cPtr->name, vector));
    Q_EMIT inputChanged(assignment);
  }
}

void InputVectorWidget::dynamicFrameChanged() {
    dynamicFrameChanged(ui_->cbTargetFrame->lineEdit()->text());
}

void InputVectorWidget::dynamicRefFrameChanged() {
    dynamicRefFrameChanged(ui_->cbSourceFrame->lineEdit()->text());
}

void InputVectorWidget::dynamicFrameChanged(QString newTarget) {
    std::string newValue = newTarget.toStdString();
    cout << "New target frame: " << newValue << endl;
    boost::shared_ptr<VectorPositionAssignment> ptr = boost::dynamic_pointer_cast<VectorPositionAssignment>(assignment);
    if (ptr && newValue != ptr->object) {
        assignment = ValuePtr(new VectorPositionAssignment(assignment->name, newValue, ptr->target));
        Q_EMIT inputChanged(assignment);
    }
}

void InputVectorWidget::dynamicRefFrameChanged(QString newSource) {
    std::string newValue = newSource.toStdString();
    cout << "New source frame: " << newValue << endl;
    boost::shared_ptr<VectorPositionAssignment> ptr = boost::dynamic_pointer_cast<VectorPositionAssignment>(assignment);
    if (ptr && newValue != ptr->target) {
        assignment = ValuePtr(new VectorPositionAssignment(assignment->name, ptr->object, newValue));
        Q_EMIT inputChanged(assignment);
    }
}

void InputVectorWidget::setConstant(bool bConst) {
  if (bConst) {
    ui_->dynamicWidget->setVisible(false);
    ui_->constVector->setVisible(true);
  
    assignment = ValuePtr(new ConstVectorAssignment(assignment->name, ui_->constVector->getVectorEigen()));
    Q_EMIT inputChanged(assignment);
  } else {
    ui_->dynamicWidget->setVisible(true);
    ui_->constVector->setVisible(false);

    // TODO: Parse ui input
    assignment = ValuePtr(new VectorPositionAssignment(assignment->name, ui_->cbTargetFrame->lineEdit()->text().toStdString(), ui_->cbSourceFrame->lineEdit()->text().toStdString()));
    Q_EMIT inputChanged(assignment);
  }
}


}
