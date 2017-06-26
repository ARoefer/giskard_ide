#pragma once

#include <QWidget>

namespace Ui
{
class SceneWidget;
}

namespace giskard_sim
{

class SceneWidget : public QWidget
{
Q_OBJECT
public:
  SceneWidget( QWidget* parent = 0 );

  ~SceneWidget();

protected:
  Ui::SceneWidget *ui_;

private:

private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
};

}  // giskard_sim
