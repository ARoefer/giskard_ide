#include "giskard_sim/pkg_combo_box.h"
#include <algorithm> // for std::sort
#include <ros/package.h>

namespace giskard_sim {
    PkgComboBox::PkgComboBox(QWidget* parent)
    : rviz::EditableComboBox(parent)
	{ }

    void PkgComboBox::showPopup() {
		if (packages.size() == 0) {
			ros::package::getAll(packages);

			for( size_t i = 0; i < packages.size(); i++ ) {
				addItem( QString::fromStdString(packages[i]));
			}
		}
        EditableComboBox::showPopup();
	}
}
