#include "giskard_sim/controller_runner.h"

#include <giskard_core/giskard_core.hpp>

namespace giskard_sim {

	ControllerRunner::ControllerRunner(const ros::Publisher& commandPublisher)
	: scenario(0)
	, controller(0)
	, valid(false)
	, initialized(false)
	, nWSR(1000)
	, cmdPub(commandPublisher)
	{ }

    void ControllerRunner::setScenario(IScenarioInstance* pScenario) {
		scenario = pScenario;
		scenario->addControllerListener(this);
	}

	void ControllerRunner::onControllerLoaded(giskard_core::QPController* _controller) {
		valid = true;
		initialized = false;
		controller = _controller;
		state = Eigen::VectorXd::Zero(controller->get_input_size());
		controllerInputs = controller->get_input_map();
	}

	void ControllerRunner::onControllerLoadFailed(const std::string& msg) {
		valid = initialized = false;
	}

	AF ControllerRunner::updateController(const sensor_msgs::JointState& jointState) {
		if(!valid)
			return AF(AF::Failure, "Can't update because controller is invalid.");

		const SScenarioContext* context = scenario->getContext();


		for (size_t i = 0; i < jointState.name.size(); i++) {
			auto it = controllerInputs.find(jointState.name[i]);
			if (it != controllerInputs.end() && it->second->get_type() == giskard_core::tJoint) {
				controller->set_input(state, jointState.name[i], jointState.position[i]);
			}
		}

		try {
			for (auto it = context->inputAssignments.begin(); it != context->inputAssignments.end(); it++) {
				auto iit = controllerInputs.find(it->first);
				if (iit != controllerInputs.end()) {
					if (iit->second->get_type() == it->second->getType()) {
						switch(iit->second->get_type()) {
							case giskard_core::tScalar:
							controller->set_input(state, it->first, static_cast<IScalarAssignment*>(it->second.get())->getValue());
							break;
							case giskard_core::tVector3:
							controller->set_input(state, it->first, static_cast<IVectorAssignment*>(it->second.get())->getValue());
							break;
							case giskard_core::tRotation:
							controller->set_input(state, it->first, static_cast<IRotationAssignment*>(it->second.get())->getValue());
							break;
							case giskard_core::tFrame:
							controller->set_input(state, it->first, static_cast<IFrameAssignment*>(it->second.get())->getValue());
							break;
							default:
							AF(AF::Failure, "Setting of joint inputs via query is not allowed.");
						}
					} else {
						return AF(AF::Failure, "Mismatched type on input '" + it->first + "'");
					}
				} else {
					return AF(AF::Failure, "Controller has no input '" + it->first + "'");
				}
			}
		} catch (const std::runtime_error& e) {
			return	AF(AF::Failure, e.what());
		}

		if (!initialized) {
			if (controller->start(state, nWSR)) {
				initialized = true;
			} else {
				return AF(AF::Failure, "Starting of controller failed!");
			}
		}

        if (controller->update(state, nWSR)) {
            auto commands = controller->get_command_map();

			sensor_msgs::JointState cmd;
            cmd.name.reserve(commands.size());
            cmd.position.reserve(commands.size());
            cmd.velocity.reserve(commands.size());
            cmd.effort.reserve(commands.size());

            for (auto it = commands.begin(); it != commands.end(); it++) {
				cmd.name.push_back(it->first);
				cmd.velocity.push_back(it->second);
                cmd.position.push_back(0);
                cmd.effort.push_back(0);
			}

			cmdPub.publish(cmd);
			return AF();
		}

		return AF(AF::Failure, "Updating of controller failed.");
	}

}
