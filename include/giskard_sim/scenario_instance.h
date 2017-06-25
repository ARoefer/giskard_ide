#pragma once

#include <urdf/model.h>
#include <ros/ros.h>
#include "giskard_core/giskard_core.hpp"
#include "giskard_sim/controller_runner.h"

#include <tf/transform_listener.h>

#include <unordered_set>

using namespace std;

namespace giskard_sim {
	class ScenarioInstance : public IScenarioInstance {
        public:
        ScenarioInstance();
		AF loadFromYAML(string path);
        AF renameScenario(std::string newName);

		AF makeURDFRelative(bool bRelative);
        AF changeURDFPath(string& newPath);
        const urdf::Model* getURDFModel() {
        	return &urdfModel;
        }

        AF makeControllerRelative(bool bRelative);
        AF changeControllerPath(std::string& newPath);
        AF reloadController() {
                AF reloadResult = loadController();
                if(!reloadResult)
                    return reloadResult;
                return resetSim();
        };
        const giskard_core::QPController* getController() const {
        	return &controller;
        }


        AF addEmptyPose();
        AF addCurrentPose();
        AF removePose(string name);
        AF setPose(string name);
        AF getPose(string name, SPose& out);
        AF renamePose(std::string pose, std::string newName);

        AF setJointPosition(std::string pose, std::string joint, double position);
        AF renameJoint(std::string pose, std::string joint, std::string newName);
        AF addJoint(std::string pose, std::string joint, double name);
        AF removeJoint(string pose, string joint);

        AF setSimTimestep(double dt);
        AF setSimDefaultPose(std::string pose);
        AF setSimUseTimestep(bool bUse);
        AF setSimState(bool bRunning);
        AF resetSim();

        template<typename T>
        T getObjectProperty(const std::string& object, const std::string& property) const {
                throw std::runtime_error("Not implemented yet.");
        }
        
        double getScalarObjectProperty(std::string object, std::string property) const {
        	return getObjectProperty<double>(object, property);
        }

        Eigen::Vector3d getVectorObjectProperty(std::string object, std::string property) const {
        	return getObjectProperty<Eigen::Vector3d>(object, property);
        }

        Eigen::Affine3d getObjectTransform(std::string object, std::string target) const;

        const SScenarioContext* getContext() const {
			return &context;
		}

                const ros::Publisher& getCommandPublisher() const {
			return cmdPublisher;
		}

        virtual void update(const ros::TimerEvent& event);

        void jointStateCB(const sensor_msgs::JointState& js);

		// ====== Listeners ========

        void addURDFListener(IURDFListener* pList);
		void removeURDFListener(IURDFListener* pList);
		
		void addControllerListener(IControllerListener* pList);
		void removeControllerListener(IControllerListener* pList);

		void addPoseListener(IPoseListener* pList);
		void removePoseListener(IPoseListener* pList);

		void addErrorListener(IErrorListener* pList);
		void removeErrorListener(IErrorListener* pList);
	
		void addScenarioListener(IScenarioListener* pList);
		void removeScenarioListener(IScenarioListener* pList);

	protected:
		virtual void notifyURDFLoaded();
		virtual void notifyControllerLoaded();

		virtual void notifyScenarioLoaded(string path);

		virtual void notifyLoadScenarioFailed(string msg);
		virtual void notifyLoadURDFFailed(string msg);
		virtual void notifyLoadControllerFailed(string msg);

		virtual void notifyPoseAdded(string pose);
		virtual void notifyPoseRemoved(string pose);
		virtual void notifyPosesCleared();
		virtual void notifyPosesLoaded();
		virtual void notifyPoseRenamed(string oldName, string newName);

		// ====== Members ===========


		sensor_msgs::JointState lastJointState;
		ControllerRunner runner;
		ros::NodeHandle nh;
		giskard_core::QPController controller;
		urdf::Model urdfModel;
		SScenarioContext context;
		ros::Publisher cmdPublisher;
		ros::Subscriber jsSubscriber;
		ros::Timer updateTimer;
		tf::TransformListener tfListener;
	private:
		AF loadController();
        AF makePathRelative(SFilePath& path, bool bRelative);

		unordered_set<IScenarioListener*> scenarioListeners;
		unordered_set<IURDFListener*> urdfListeners;
		unordered_set<IControllerListener*> controllerListeners;
		unordered_set<IErrorListener*> errorListeners;
		unordered_set<IPoseListener*> poseListeners;
	};
}
