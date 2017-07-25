#include "giskard_sim/ros_pkg_file_selector.h"

#include "ui_ros_pkg_file_selector.h"

using namespace std;

namespace giskard_sim
{

RosPkgFileSelector::RosPkgFileSelector( QWidget* parent )
  : QWidget( parent )
  , ui_(new Ui::RosPkgFileSelector())
{
  // set up the GUI
  ui_->setupUi(this);

  ui_->cbFile->setEnabled(false);
}

RosPkgFileSelector::~RosPkgFileSelector() {
    delete ui_;
}

void RosPkgFileSelector::addSuffix(std::string suffix) {
    ui_->cbFile->addSuffix(suffix);
}

void RosPkgFileSelector::setValue(std::string path) {
    if (path.find("package://") == 0) {
        std::string temp = path.substr(10);
        std::string pkg  = temp.substr(0, temp.find('/'));
        std::string file = temp.substr(temp.find('/'));
        ui_->cbPkg->setEditText(QString::fromStdString(pkg));
        ui_->cbFile->setPackage(pkg);
        ui_->cbFile->setEditText(QString::fromStdString(file));
    } else {
        ui_->cbPkg->setEditText("");
        ui_->cbFile->setEditText("");
        ui_->cbFile->setEnabled(false);
    }
}

std::string RosPkgFileSelector::getValue() const {
    std::string pkg = ui_->cbPkg->currentText().toStdString();
    if (pkg.empty())
        return "";

    return "package://" + pkg + '/' + ui_->cbFile->currentText().toStdString();
}

void RosPkgFileSelector::packageChanged(QString newPkg) {
    ui_->cbFile->setEditText("");
    ui_->cbFile->setEnabled(newPkg.size() != 0);
    if (newPkg.size() != 0)
        ui_->cbFile->setPackage(newPkg.toStdString());
}

void RosPkgFileSelector::fileChanged(QString newFile) {
    Q_EMIT fileChanged(getValue());   
}

}
