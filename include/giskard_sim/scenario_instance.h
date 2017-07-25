#pragma once

#include <urdf/model.h>
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>

#include "giskard_core/giskard_core.hpp"
#include "giskard_sim/controller_runner.h"

#include <tf/transform_listener.h>

#include <unordered_set>
#include <unordered_map>

using namespace std;

namespace giskard_sim {
	class ScenarioInstance : public IScenarioInstance {
        static ScenarioInstance* _currentInstance;
        static void _processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

        public:
        ScenarioInstance();
        ~ScenarioInstance();
		AF loadFromYAML(string path);
        AF renameScenario(std::string newName);

		AF setPostureService(std::string serviceName);
		AF setJointStateTopic(std::string topicName);
		AF setCommandTopic(std::string topicName);

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

		AF addSceneObject(SWorldObject* pObject);
		AF removeSceneObject(std::string objectName);
		AF selectSceneObject(std::string objectName);
		AF attachSceneObject(std::string objectName, std::string frame, bool keepTransform);

		AF setInputAssignment(boost::shared_ptr<IInputAssignment> assignment);

        virtual void update(const ros::TimerEvent& event);

        void jointStateCB(const sensor_msgs::JointState& js);

		// ====== Listeners ========

        void addURDFListener(IURDFListener* pList);
		void removeURDFListener(IURDFListener* pList);
		
		void addTopicListener(ITopicListener* pList);
		void removeTopicListener(ITopicListener* pList);

		void addControllerListener(IControllerListener* pList);
		void removeControllerListener(IControllerListener* pList);

		void addPoseListener(IPoseListener* pList);
		void removePoseListener(IPoseListener* pList);

		void addErrorListener(IErrorListener* pList);
		void removeErrorListener(IErrorListener* pList);
	
		void addScenarioListener(IScenarioListener* pList);
		void removeScenarioListener(IScenarioListener* pList);

	protected:
		virtual AF validateInputAssignments();

		virtual void notifyURDFLoaded();
		virtual void notifyControllerLoaded();

		virtual void notifyTopicsChanged();

		virtual void notifyScenarioLoaded(string path);
		virtual void notifyObjectAdded(const SWorldObject& object);
		virtual void notifyObjectChanged(const SWorldObject& object);
		virtual void notifyObjectRemoved(const std::string& name);
		virtual void notifySelectedObjectChanged(const std::string& selected);

		virtual void notifyLoadScenarioFailed(string msg);
		virtual void notifyLoadURDFFailed(string msg);
		virtual void notifyLoadControllerFailed(string msg);
		virtual void notifyRunControllerFailed(string msg);

		virtual void notifyPoseAdded(string pose);
		virtual void notifyPoseRemoved(string pose);
		virtual void notifyPosesCleared();
		virtual void notifyPosesLoaded();
		virtual void notifyPoseRenamed(string oldName, string newName);

		virtual void notifyInputAssignmentChanged(AssignmentPtr assignment);
		virtual void notifyInputAssignmentDeleted(string name);
		virtual void notifyInputAssignmentsLoaded();
		virtual void notifyInputAssignmentsCleared();

		virtual void processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

		// ====== Members ===========

        interactive_markers::InteractiveMarkerServer intServer;

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
		std::string selectedObject;
	private:
		AF loadController();
        AF makePathRelative(SFilePath& path, bool bRelative);

		unordered_set<IScenarioListener*> scenarioListeners;
		unordered_set<ITopicListener*> topicListeners;
		unordered_set<IURDFListener*> urdfListeners;
		unordered_set<IControllerListener*> controllerListeners;
		unordered_set<IErrorListener*> errorListeners;
		unordered_set<IPoseListener*> poseListeners;
	};
}
