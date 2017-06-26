#pragma once

#include <QWidget>
#include <eigen3/Eigen/Eigen>

namespace Ui
{
class ObjectInfoWidget;
}

namespace giskard_sim
{

class ObjectInfoWidget : public QWidget
{
Q_OBJECT
public:
  ObjectInfoWidget( QWidget* parent = 0 );

  ~ObjectInfoWidget();

protected:
  Ui::ObjectInfoWidget *ui_;

private:
  
private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void onScaleChanged(Eigen::Vector3d scale);
  void onPositionChanged(Eigen::Vector3d position);
  void onRotationChanged(Eigen::Vector3d rotation);
  void onColorChanged();
  void onParentChanged(QString frame);
  void onVisualChanged(int type);
  void onNameChanged();

};

}  // giskard_sim
