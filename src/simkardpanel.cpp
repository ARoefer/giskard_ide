#include "giskard_sim/simkardpanel.h"
#include "ui_simkardpanel.h"

SimkardPanel::SimkardPanel(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::SimkardPanel)
{
    ui->setupUi(this);
}

SimkardPanel::~SimkardPanel()
{
    delete ui;
}
