#include "giskard_sim/create_object_dialog.h"

#include "ui_create_object_dialog.h"

using namespace std;

namespace giskard_sim
{

CreateObjectDialog::CreateObjectDialog( QWidget* parent )
  : QDialog( parent )
  , ui_(new Ui::CreateObjectDialog())
{
  // set up the GUI
  ui_->setupUi(this);
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
