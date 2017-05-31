#ifndef SIMKARDPANEL_H
#define SIMKARDPANEL_H

#include <rviz/panel.h>

namespace Ui {
class SimkardPanel;
}

class SimkardPanel : public rviz::Panel
{
    Q_OBJECT

public:
    explicit SimkardPanel(QWidget *parent = 0);
    ~SimkardPanel();

private:
    Ui::SimkardPanel *ui;
};

#endif // SIMKARDPANEL_H
