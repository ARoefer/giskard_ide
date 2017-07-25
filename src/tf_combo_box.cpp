#include "giskard_sim/tf_combo_box.h"
#include <algorithm> // for std::sort

namespace giskard_sim {
    TfFrameComboBox::TfFrameComboBox(QWidget* parent, rviz::FrameManager* _frameManager)
    : rviz::EditableComboBox(parent)
	{ 
		setFrameManager(_frameManager);
	}

    void TfFrameComboBox::setFrameManager(rviz::FrameManager* fm) {
		frameManager = fm;
		if(!frameManager) {
			clear();
		}
	}

    void TfFrameComboBox::showPopup() {
		clear();
		if (frameManager) {
			std::vector<std::string> stdFrames;
			frameManager->getTFClient()->getFrameStrings( stdFrames );
			std::sort( stdFrames.begin(), stdFrames.end() );

			for( size_t i = 0; i < stdFrames.size(); i++ ) {
				addItem( QString::fromStdString(stdFrames[ i ]));
			}			
		}
        EditableComboBox::showPopup();
	}
}
