#pragma once

#include <QWidget>
#include <eigen3/Eigen/Eigen>

namespace Ui
{
class VectorWidget;
}

namespace giskard_sim
{

class VectorWidget : public QWidget
{
Q_OBJECT
public:
  VectorWidget( QWidget* parent = 0 );

  ~VectorWidget();

  virtual void showScaleSlider(bool bShow);

  virtual void setLabelTexts(QString lX, QString lY, QString lZ);
  void setLabelTexts(std::string lX, std::string lY, std::string lZ);

  virtual void setVector(Eigen::Vector3d vector);
  virtual Eigen::Vector3d getVectorEigen();

Q_SIGNALS:
  void valueChanged(Eigen::Vector3d newValue);

protected:
  Ui::VectorWidget *ui_;

private:
  double x, y, z;
  double scaleFactor;
  bool bInScaleMode;

private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void onXChanged();
  void onYChanged();
  void onZChanged();
  void onSliderMoved(int amount);
  void onScaleBegin();
  void onScaleEnd();
};

}  // giskard_sim
