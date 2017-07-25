#pragma once

#include "rviz/frame_manager.h"
#include "rviz/properties/editable_combo_box.h"

namespace giskard_sim {

        class TfFrameComboBox : public rviz::EditableComboBox {
	Q_OBJECT
	public:
                TfFrameComboBox(QWidget* parent = 0, rviz::FrameManager* frameManager = 0);
		void showPopup();
		void setFrameManager(rviz::FrameManager* frame_manager);
	private:
		rviz::FrameManager* frameManager;
	};
}
