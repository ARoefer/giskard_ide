#pragma once

#include <QWidget>
#include <eigen3/Eigen/Eigen>
#include <boost/shared_ptr.hpp>

namespace Ui
{
class InputVectorWidget;
}

namespace rviz {
  class FrameManager;
}

namespace giskard_sim
{
struct IInputAssignment;
class IVectorAssignment;

class InputVectorWidget : public QWidget
{
Q_OBJECT
public:
    typedef typename boost::shared_ptr<giskard_sim::IVectorAssignment> ValuePtr;
  InputVectorWidget( QWidget* parent, ValuePtr value, rviz::FrameManager* frameManager = 0);

  ~InputVectorWidget();

  virtual void setFrameManager(rviz::FrameManager* frameManager);
  virtual void setValue(ValuePtr value);
  
  virtual ValuePtr getInput() const { return assignment; }

Q_SIGNALS:
  void inputChanged(boost::shared_ptr<IInputAssignment> value);

protected:
  Ui::InputVectorWidget *ui_;

private:
  ValuePtr assignment;
  rviz::FrameManager* frameManager;

private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void constantVectorChanged(Eigen::Vector3d vector);
  void dynamicFrameChanged();
  void dynamicRefFrameChanged();
  void dynamicFrameChanged(QString newVec);
  void dynamicRefFrameChanged(QString newVec);
  void setConstant(bool bConst);
};

}  // giskard_sim
