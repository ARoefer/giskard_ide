#pragma once

#include <giskard_sim/interfaces.h>
#include <rviz/default_plugin/robot_model_display.h>
#include <urdf/model.h>
#include <rviz/robot/robot.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/status_property.h>
#include <rviz/robot/tf_link_updater.h>

namespace giskard_sim {
void linkUpdaterStatusFunction( rviz::StatusProperty::Level level,
                                const std::string& link_name,
                                const std::string& text,
                                rviz::RobotModelDisplay* display ) {
    display->setStatus( level, QString::fromStdString( link_name ), QString::fromStdString( text ));
}

class RobotModelDisplay : public rviz::RobotModelDisplay, public IURDFListener {
public:
        void onURDFChanged(const urdf::Model* model) {
            if(!model) {
                //robot_->clear();
                return;
            }

            robot_->load(*model);
            robot_->update( rviz::TFLinkUpdater( context_->getFrameManager(),
                                           boost::bind( linkUpdaterStatusFunction, _1, _2, _3, this ),
                                           tf_prefix_property_->getStdString() ));
            robot_->setVisible(true);
	}
};
	
}
