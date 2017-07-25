#pragma once

#include <QWidget>
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Eigen>

namespace Ui
{
class InputRotationWidget;
}

namespace rviz {
  class FrameManager;
}

namespace giskard_sim
{
struct IInputAssignment;
class IRotationAssignment;

class InputRotationWidget : public QWidget
{
Q_OBJECT
public:
  typedef typename boost::shared_ptr<giskard_sim::IRotationAssignment> ValuePtr;
  InputRotationWidget( QWidget* parent, ValuePtr value, rviz::FrameManager* frameManager = 0);

  ~InputRotationWidget();

  virtual void setFrameManager(rviz::FrameManager* frameManager);
  virtual void setValue(ValuePtr value);
  
  virtual ValuePtr getInput() const { return assignment; }

Q_SIGNALS:
  void inputChanged(boost::shared_ptr<IInputAssignment> value);

protected:
  Ui::InputRotationWidget *ui_;
  rviz::FrameManager* frameManager;

private:
  ValuePtr assignment;

private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void constantRotationChanged(Eigen::Vector3d);
  void dynamicFrameChanged();
  void dynamicRefFrameChanged();
  void dynamicFrameChanged(QString newVec);
  void dynamicRefFrameChanged(QString newVec);
  void setConstant(bool bConst);
};

}  // giskard_sim
