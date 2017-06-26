#include "giskard_sim/scene_widget.h"
#include "giskard_sim/utils.h"

#include "ui_scene_widget.h"

using namespace std;

namespace giskard_sim
{

SceneWidget::SceneWidget( QWidget* parent )
  : QWidget( parent )
  , ui_(new Ui::SceneWidget())
{
  // set up the GUI
  ui_->setupUi(this);
}

SceneWidget::~SceneWidget() {
    delete ui_;
}

}
