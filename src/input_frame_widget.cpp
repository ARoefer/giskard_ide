#include "giskard_sim/input_frame_widget.h"
#include "giskard_sim/utils.h"
#include "giskard_sim/decisive_double_validator.h"

#include "ui_input_frame_widget.h"

#include <rviz/frame_manager.h>

#include <QLineEdit>

using namespace std;

namespace giskard_sim
{

InputFrameWidget::InputFrameWidget( QWidget* parent, ValuePtr _assignment, rviz::FrameManager* _frameManager)
  : QWidget( parent )
  , ui_(new Ui::InputFrameWidget())
  , assignment(_assignment)
{
  // set up the GUI
  ui_->setupUi(this);
  ui_->labelName->setText(QString::fromStdString(assignment->name));
  ui_->constRotation->showScaleSlider(false);
  ui_->constPosition->showScaleSlider(false);

  setFrameManager(_frameManager);

  connect(ui_->cbTargetFrame->lineEdit(), SIGNAL(editingFinished()), this, SLOT(dynamicFrameChanged()));
  connect(ui_->cbSourceFrame->lineEdit(), SIGNAL(editingFinished()), this, SLOT(dynamicRefFrameChanged()));

  setValue(assignment);
}

InputFrameWidget::~InputFrameWidget() {
    delete ui_;
}

void InputFrameWidget::setValue(ValuePtr value) {
  ui_->labelName->setText(QString::fromStdString(value->name));
  assignment = value;
  if (boost::dynamic_pointer_cast<ConstFrameAssignment>(assignment)) {
    boost::shared_ptr<ConstFrameAssignment> ptr = boost::dynamic_pointer_cast<ConstFrameAssignment>(assignment);
    ui_->dynamicWidget->setVisible(false);
    ui_->constWidget->setVisible(true);

    ui_->constRotation->setVector(toEulerDeg(ptr->getValue().rotation()));
    ui_->constPosition->setVector(ptr->getValue().translation());
    ui_->cbIsConst->setChecked(true);
  } else if (boost::dynamic_pointer_cast<FrameAssignment>(assignment)){
    boost::shared_ptr<FrameAssignment> ptr = boost::dynamic_pointer_cast<FrameAssignment>(assignment);
    ui_->dynamicWidget->setVisible(true);
    ui_->constWidget->setVisible(false);

    ui_->cbSourceFrame->lineEdit()->setText(QString::fromStdString(ptr->target));
    ui_->cbTargetFrame->lineEdit()->setText(QString::fromStdString(ptr->object));
    ui_->cbIsConst->setChecked(false);
  }
}

void InputFrameWidget::constantPositionChanged(Eigen::Vector3d pos) {
  Eigen::Affine3d newVal = makeAffine(pos, fromEulerDeg(ui_->constRotation->getVectorEigen()));

  boost::shared_ptr<ConstFrameAssignment> cPtr = boost::dynamic_pointer_cast<ConstFrameAssignment>(assignment);
  if (cPtr && cPtr->getValue() != newVal) {
    assignment = ValuePtr(new ConstFrameAssignment(cPtr->name, newVal));
    Q_EMIT inputChanged(assignment);
  }
}

void InputFrameWidget::constantRotationChanged(Eigen::Vector3d rot) {
  Eigen::Affine3d newVal = makeAffine(ui_->constPosition->getVectorEigen(), fromEulerDeg(rot));

  boost::shared_ptr<ConstFrameAssignment> cPtr = boost::dynamic_pointer_cast<ConstFrameAssignment>(assignment);
  if (cPtr) {
    assignment = ValuePtr(new ConstFrameAssignment(cPtr->name, newVal));
    Q_EMIT inputChanged(assignment);
  }
}

void InputFrameWidget::dynamicFrameChanged() {
    dynamicFrameChanged(ui_->cbTargetFrame->lineEdit()->text());
}

void InputFrameWidget::dynamicRefFrameChanged() {
    dynamicRefFrameChanged(ui_->cbSourceFrame->lineEdit()->text());
}

void InputFrameWidget::dynamicFrameChanged(QString newTarget) {
    std::string newValue = newTarget.toStdString();
    cout << "New target frame: " << newValue << endl;
    boost::shared_ptr<FrameAssignment> ptr = boost::dynamic_pointer_cast<FrameAssignment>(assignment);
    if (ptr && newValue != ptr->object) {
        assignment = ValuePtr(new FrameAssignment(assignment->name, newValue, ptr->target));
        Q_EMIT inputChanged(assignment);
    }
}

void InputFrameWidget::dynamicRefFrameChanged(QString newSource) {
    std::string newValue = newSource.toStdString();
    cout << "New source frame: " << newValue << endl;
    boost::shared_ptr<FrameAssignment> ptr = boost::dynamic_pointer_cast<FrameAssignment>(assignment);
    if (ptr && newValue != ptr->target) {
        assignment = ValuePtr(new FrameAssignment(assignment->name, ptr->object, newValue));
        Q_EMIT inputChanged(assignment);
    }
}

void InputFrameWidget::setConstant(bool bConst) {
  if (bConst) {
    ui_->dynamicWidget->setVisible(false);
    ui_->constWidget->setVisible(true);
  
    assignment = ValuePtr(new ConstFrameAssignment(assignment->name, makeAffine(ui_->constPosition->getVectorEigen(), fromEulerDeg(ui_->constRotation->getVectorEigen()))));
    Q_EMIT inputChanged(assignment);
  } else {
    ui_->dynamicWidget->setVisible(true);
    ui_->constWidget->setVisible(false);
    
    // TODO: Parse ui input
    assignment = ValuePtr(new FrameAssignment(assignment->name, ui_->cbTargetFrame->lineEdit()->text().toStdString(), ui_->cbSourceFrame->lineEdit()->text().toStdString()));
    Q_EMIT inputChanged(assignment);
  }
}

void InputFrameWidget::setFrameManager(rviz::FrameManager* frameManager) {
  ui_->cbTargetFrame->setFrameManager(frameManager);
  ui_->cbSourceFrame->setFrameManager(frameManager);
}

}
