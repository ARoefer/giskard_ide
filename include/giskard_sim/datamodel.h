#pragma once
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

#include "giskard_sim/input_assignments.h"

using namespace std;

namespace giskard_sim {

	struct SFilePath {
		SFilePath() : packageRelative(false) {}
		string path;
		bool packageRelative;
		
		string toString() const {
			if (packageRelative)
				return "package-path: "+path;
			else
				return "global-path: "+path;
		}
	};


	struct SPose {
		map<string, double> jointPos;
		string name;
		string robotName;

		sensor_msgs::JointState toJointState() {
			sensor_msgs::JointState out;
			for (auto it = jointPos.begin(); it != jointPos.end(); it++) {
				out.name.push_back(it->first);
				out.position.push_back(it->second);
				out.velocity.push_back(0);
				out.effort.push_back(0);
			}
			return out;
		}
	};

	struct SSimSettings {
		SSimSettings() : timeStep(0.025), bUseTimeStep(false), bRunning(false) {}
		double timeStep;
		bool bUseTimeStep;
		bool bRunning;
		string defaultPose;
	};

	struct SWorldObject {
		string parent;
		Eigen::Affine3d transform;
		visualization_msgs::Marker visual;
		YAML::Node properties;
	};

	struct SScenarioContext {
		SScenarioContext() 
		: name("My Scenario")
		, jsTopic("joint_states")
		, cmdTopic("/simulator/commands")
		, setJSService("/simulator/set_joint_states")
		{}

		string name;

		SFilePath controllerPath;
		SFilePath urdfPath;
		SSimSettings simSettings;
		map<string, SPose> poses;
		map<string, shared_ptr<IInputAssignment>> inputAssignments;
		map<string, shared_ptr<SWorldObject>> objects;

		// Topics
		string jsTopic;
		string cmdTopic;

		// Services
		string setJSService;
	};
}

namespace YAML {
	template<>
	struct convert<Eigen::Vector3d>
	{
		static Node encode(const Eigen::Vector3d& v) {
			Node node;
			node["x"] = v[0];
			node["y"] = v[1];
			node["z"] = v[2];
			return node;
		}

		static bool decode(const Node& node, Eigen::Vector3d& v) {
			try {
				v[0] = node["x"].as<double>();
				v[1] = node["y"].as<double>();
				v[2] = node["z"].as<double>();
				return true;
			} catch (const YAML::Exception& e) {
				return false;
			}
		}
	};

	template<>
	struct convert<giskard_sim::SFilePath>
	{
		static Node encode(const giskard_sim::SFilePath& fp) {
			Node node;
			node["path"] = fp.path;
			node["package"] = fp.packageRelative;
			return node;
		}

		static bool decode(const Node& node, giskard_sim::SFilePath& fp) {
			try {
				fp.path = node["path"].as<string>();
				fp.packageRelative = node["package"].as<bool>();
				return true;
			} catch (const YAML::Exception& e) {
				return false;
			}
		}
	};

	template<>
	struct convert<shared_ptr<giskard_sim::IInputAssignment>>
	{
		static Node encode(const shared_ptr<giskard_sim::IInputAssignment>& pIA) {
			return pIA->toYAML();
		}

		static bool decode(const Node& node, shared_ptr<giskard_sim::IInputAssignment>& pIA) {
			try {
				string type = node["type"].as<string>();

				if (type == "constScalar")
					pIA = shared_ptr<giskard_sim::IInputAssignment>(new giskard_sim::ConstScalarAssignment(node["value"].as<double>()));
				else if (type == "scalarProperty")
					pIA = shared_ptr<giskard_sim::IInputAssignment>(new giskard_sim::ScalarPropertyAssignment(node["object"].as<string>(), node["property"].as<string>()));
				else if (type == "constVector")
					pIA = shared_ptr<giskard_sim::IInputAssignment>(new giskard_sim::ConstVectorAssignment(node["value"].as<Eigen::Vector3d>()));
				else if (type == "vectorProperty")
					pIA = shared_ptr<giskard_sim::IInputAssignment>(new giskard_sim::VectorPropertyAssignment(node["object"].as<string>(), node["property"].as<string>()));
				else if (type == "positionQuery")
					pIA = shared_ptr<giskard_sim::IInputAssignment>(new giskard_sim::VectorPositionAssignment(node["object"].as<string>(), node["target"].as<string>()));
				else if (type == "rotationQuery")
					pIA = shared_ptr<giskard_sim::IInputAssignment>(new giskard_sim::RotationAssignment(node["object"].as<string>(), node["target"].as<string>()));
				else if (type == "frameQuery")
					pIA = shared_ptr<giskard_sim::IInputAssignment>(new giskard_sim::FrameAssignment(node["object"].as<string>(), node["target"].as<string>()));
				return true;
			} catch (const YAML::Exception& e) {
				return false;
			}
		}
	};

	template<>
	struct convert<giskard_sim::SPose>
	{
		static Node encode(const giskard_sim::SPose& p) {
			Node node;
			node["name"] = p.name;
			node["robotName"] = p.robotName;
			node["positions"] = p.jointPos;

			return node;
		}

		static bool decode(const Node& node, giskard_sim::SPose& p) {
			try {
				p.name = node["name"].as<string>();
				p.robotName = node["robotName"].as<string>();
				p.jointPos = node["positions"].as<map<string, double>>();

				return true;
			} catch (const YAML::Exception& e) {
				return false;
			}
		}
	};

	template<>
	struct convert<giskard_sim::SSimSettings>
	{
		static Node encode(const giskard_sim::SSimSettings& ss) {
			Node node;
			node["timeStep"] = ss.timeStep;
			node["useTimeStep"] = ss.bUseTimeStep;
			node["isRunning"] = ss.bRunning;
			node["defaultPose"] = ss.defaultPose;
			return node;
		}

		static bool decode(const Node& node, giskard_sim::SSimSettings& ss) {
			try {
				ss.timeStep = node["timeStep"].as<double>();
				ss.bUseTimeStep = node["useTimeStep"].as<bool>();
				ss.bRunning = node["isRunning"].as<bool>();
				ss.defaultPose = node["defaultPose"].as<string>();
				return true;
			} catch (const YAML::Exception& e) {
				return false;
			}
		}
	};

	template<>
	struct convert<giskard_sim::SScenarioContext>
	{
		static Node encode(const giskard_sim::SScenarioContext& sc) {
			Node node;
			node["name"] = sc.name;
			node["controllerPath"] = sc.controllerPath;
			node["urdfPath"] = sc.urdfPath;
			node["poses"] = sc.poses;
			node["topic_jointStates"] = sc.jsTopic;
			node["topic_jointCommands"] = sc.cmdTopic;
			node["service_setJointState"] = sc.setJSService;
			node["simulationSettings"] = sc.simSettings;
			node["inputAssignments"] = sc.inputAssignments;
			return node;
		}

		static bool decode(const Node& node, giskard_sim::SScenarioContext& sc) {
			try {
				sc.name             = node["name"].as<string>();
				sc.controllerPath   = node["controllerPath"].as<giskard_sim::SFilePath>();
				sc.urdfPath         = node["urdfPath"].as<giskard_sim::SFilePath>();
				sc.poses            = node["poses"].as<map<string, giskard_sim::SPose>>();
				sc.jsTopic          = node["topic_jointStates"].as<string>();
				sc.cmdTopic         = node["topic_jointCommands"].as<string>();
				sc.setJSService     = node["service_setJointState"].as<string>();
				sc.simSettings      = node["simulationSettings"].as<giskard_sim::SSimSettings>();
				sc.inputAssignments = node["inputAssignments"].as<map<string, shared_ptr<giskard_sim::IInputAssignment>>>();
				return true;
			} catch (const YAML::Exception& e) {
				return false;
			}
		}
	};
}
