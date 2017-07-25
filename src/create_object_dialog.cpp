#include "giskard_sim/create_object_dialog.h"
#include <rviz/frame_manager.h>

#include "ui_create_object_dialog.h"

using namespace std;

namespace giskard_sim
{

CreateObjectDialog::CreateObjectDialog( QWidget* parent, rviz::FrameManager* fM)
  : QDialog( parent )
  , ui_(new Ui::CreateObjectDialog())
{
  // set up the GUI
  ui_->setupUi(this);
  ui_->objectInfo->setFrameManager(fM);
}

CreateObjectDialog::~CreateObjectDialog() {
    delete ui_;
}

void CreateObjectDialog::accept() {
  Q_EMIT createObject(ui_->objectInfo->getObject());
    cout << "Closing dialog" << endl;
    QDialog::accept();
}

void CreateObjectDialog::reject() {
    cout << "Closing dialog" << endl;
    QDialog::reject();
}

}
