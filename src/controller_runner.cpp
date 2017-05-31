#include "giskard_sim/controller_runner.h"

#include <giskard/giskard.hpp>

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

	void ControllerRunner::onControllerLoaded(giskard::QPController* _controller) {
		valid = true;
		initialized = false;
		controller = _controller;
		state = Eigen::VectorXd::Zero(controller->get_scope().get_input_size());
		controllerInputs = controller->get_scope().get_inputs();
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
			if (it != controllerInputs.end() && it->second.type == giskard::Scope::Joint) {
				controller->get_scope().set_input(jointState.name[i], jointState.position[i], state);
			}
		}

		try {
			for (auto it = context->inputAssignments.begin(); it != context->inputAssignments.end(); it++) {
				auto iit = controllerInputs.find(it->first);
				if (iit != controllerInputs.end()) {
					if (iit->second.type == it->second->getType()) {
						switch(iit->second.type) {
							case giskard::Scope::Scalar:
							controller->get_scope().set_input(it->first, static_cast<IScalarAssignment*>(it->second.get())->getValue(), state);
							break;
							case giskard::Scope::Vector:
							controller->get_scope().set_input(it->first, static_cast<IVectorAssignment*>(it->second.get())->getValue(), state);
							break;
							case giskard::Scope::Rotation:
							controller->get_scope().set_input(it->first, static_cast<IRotationAssignment*>(it->second.get())->getValue(), state);
							break;
							case giskard::Scope::Frame:
							controller->get_scope().set_input(it->first, static_cast<IFrameAssignment*>(it->second.get())->getValue(), state);
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
            const Eigen::VectorXd& command = controller->get_command();
			const std::vector<std::string>& cjoints =  controller->get_controllable_names();

			sensor_msgs::JointState cmd;
            cmd.name.reserve(cjoints.size());
            cmd.position.reserve(cjoints.size());
            cmd.velocity.reserve(cjoints.size());
            cmd.effort.reserve(cjoints.size());

            for (size_t i = 0; i < cjoints.size(); i++) {
				cmd.name.push_back(cjoints[i]);
				cmd.velocity.push_back(command[i]);
                cmd.position.push_back(0);
                cmd.effort.push_back(0);
			}

			cmdPub.publish(cmd);
			return AF();
		}

		return AF(AF::Failure, "Updating of controller failed.");
	}

}
