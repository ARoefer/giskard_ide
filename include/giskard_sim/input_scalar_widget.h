#pragma once

#include <QWidget>
#include <boost/shared_ptr.hpp>

namespace Ui
{
class InputScalarWidget;
}

namespace giskard_sim
{
struct IInputAssignment;
class IScalarAssignment;

class InputScalarWidget : public QWidget
{
Q_OBJECT
public:
  typedef typename boost::shared_ptr<giskard_sim::IScalarAssignment> ValuePtr;
  InputScalarWidget( QWidget* parent, ValuePtr value );

  ~InputScalarWidget();

  virtual void setValue(ValuePtr value);
  
  virtual ValuePtr getInput() const { return assignment; }

Q_SIGNALS:
  void inputChanged(boost::shared_ptr<IInputAssignment> value);

protected:
  Ui::InputScalarWidget *ui_;

private:
  ValuePtr assignment;

private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void constantScalarEdited();
  void dynamicScalarChanged(QString newVec);
  void setConstant(bool bConst);
};

}  // giskard_sim
