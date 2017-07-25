#pragma once

#include "rviz/frame_manager.h"
#include "rviz/properties/editable_combo_box.h"

namespace giskard_sim {

        class PkgComboBox : public rviz::EditableComboBox {
	Q_OBJECT
	public:
                PkgComboBox(QWidget* parent = 0);
		void showPopup();
	private:
		std::vector<std::string> packages;
	};
}
