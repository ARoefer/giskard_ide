#pragma once
#include "giskard_sim/datamodel.h"

#include <QWidget>
#include <eigen3/Eigen/Eigen>

namespace Ui
{
class ObjectInfoWidget;
}

namespace rviz {
class FrameManager;
}

namespace giskard_sim
{

class ObjectInfoWidget : public QWidget
{
Q_OBJECT
public:
  ObjectInfoWidget( QWidget* parent = 0 );

  ~ObjectInfoWidget();

  virtual void setObject(SWorldObject obj);
  SWorldObject getObject() const;
  void setFrameManager(rviz::FrameManager* frameManager);

Q_SIGNALS:
  void objectEdited(giskard_sim::SWorldObject object);

protected:
  Ui::ObjectInfoWidget *ui_;

  SWorldObject object;

private:
  
private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void onScaleChanged(Eigen::Vector3d scale);
  void onPositionChanged(Eigen::Vector3d position);
  void onRotationChanged(Eigen::Vector3d rotation);
  void onMeshChanged(std::string);
  void onUseMaterialsChanged(int);
  void onColorChanged();
  void onParentMaybeChanged();
  void onParentChanged(QString frame);
  void onVisualChanged(int type);
  void onNameChanged();

};

}  // giskard_sim
