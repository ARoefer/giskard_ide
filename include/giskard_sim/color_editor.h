#pragma once

#include "rviz/properties/color_editor.h"
#include "rviz/properties/color_property.h"

namespace giskard_sim {
	class ColorEditor : public rviz::ColorEditor {
		Q_OBJECT
	public:
		ColorEditor(QWidget* parent = 0)
		: prop()
		, rviz::ColorEditor(&prop, parent) {}
	
	private:
		rviz::ColorProperty prop;
	};
}
