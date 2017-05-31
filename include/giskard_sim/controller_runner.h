#pragma once
#include "giskard_sim/datamodel.h"
#include <ros/ros.h>

namespace giskard_sim {

	class ControllerRunner : public IScenarioReference, public IControllerListener {
	public:
		ControllerRunner(const ros::Publisher& commandPublisher);

		void setScenario(IScenarioInstance* pScenario);

		void onControllerLoaded(giskard::QPController* _controller);
		void onControllerLoadFailed(const std::string& msg);

		inline bool isValid() {
			return valid;
		}

		AF updateController(const sensor_msgs::JointState& jointState);

	private:
		const ros::Publisher& cmdPub;
		giskard::QPController* controller;
		std::map<const std::string, giskard::Scope::ScopeInput> controllerInputs;
		int nWSR;
		Eigen::VectorXd state;
		bool valid;
		bool initialized;
		IScenarioInstance* scenario;

	};
}
