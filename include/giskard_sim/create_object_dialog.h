#pragma once
#include "giskard_sim/datamodel.h"

#include <QDialog>

namespace rviz {
   class FrameManager;
}

namespace Ui
{
class CreateObjectDialog;
}

namespace giskard_sim
{

class CreateObjectDialog : public QDialog
{
Q_OBJECT
public:
  CreateObjectDialog( QWidget* parent = 0, rviz::FrameManager* frameManager = 0);

  ~CreateObjectDialog();

Q_SIGNALS:
  void createObject(SWorldObject object);

protected:
  Ui::CreateObjectDialog *ui_;

private:
  
private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void accept();
  void reject();

};

}  // giskard_sim
