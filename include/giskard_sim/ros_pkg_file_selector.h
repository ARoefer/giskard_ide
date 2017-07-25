#pragma once

#include <QWidget>

namespace Ui
{
class RosPkgFileSelector;
}

namespace giskard_sim
{

class RosPkgFileSelector : public QWidget
{
Q_OBJECT
public:
  RosPkgFileSelector( QWidget* parent);

  ~RosPkgFileSelector();

  void addSuffix(std::string suffix);
  virtual void setValue(std::string path);
  virtual std::string getValue() const;

Q_SIGNALS:
  void fileChanged(std::string);

protected:
  Ui::RosPkgFileSelector *ui_;

private:


private Q_SLOTS:
  // Q_SLOTS for interaction with buttons, etc.
  void packageChanged(QString);
  void fileChanged(QString);
  
};

}  // giskard_sim