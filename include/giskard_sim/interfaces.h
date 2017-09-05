#pragma once

#include <string>
#include <eigen3/Eigen/Eigen>
#include <boost/shared_ptr.hpp>
#include <map>

namespace urdf {
	class Model;
}

namespace giskard_core {
	class QPControllerSpec;
	class QPController;
}

namespace giskard_sim {
	struct SPose;
    struct SWorldObject;
    struct IInputAssignment;

	struct ActionFeedback {
		enum Type {
			Success,
			Failure,
			Error
		};

		ActionFeedback() : t(Success) {}

		ActionFeedback(Type _t) : t(_t) {}

		ActionFeedback(Type _t, std::string _msg) : t(_t), msg(_msg) {}

		const Type t;
		const std::string msg;

		operator bool() const {
			return t == Success;
		}

		operator std::string() const {
			return msg;
		}

		friend std::ostream& operator<< (std::ostream& stream, const ActionFeedback& f) {
			stream << f.msg;
			return stream;
		}
	};

	typedef ActionFeedback AF;

	struct SScenarioContext;

	struct IURDFListener {
		virtual void onURDFChanged(const urdf::Model* model) = 0;
	};
	struct IControllerListener {
		virtual void onControllerLoaded(giskard_core::QPController* controller) = 0;
		virtual void onControllerLoadFailed(const std::string& msg) = 0;
		virtual void onInputAssignmentChanged(boost::shared_ptr<IInputAssignment> assignment) = 0;
		virtual void onInputAssignmentDeleted(const std::string& inputName) = 0;
        virtual void onInputsLoaded(const std::map<std::string, boost::shared_ptr<IInputAssignment>>& inputs) = 0;
		virtual void onInputsCleared() = 0;
	};

	struct ITopicListener {
		virtual void onTopicsChanged() = 0;
	};

	struct IScenarioListener {
		virtual void onScenarioLoaded(std::string path, const SScenarioContext* context) = 0;
		virtual void onObjectAdded(const SWorldObject& object) = 0;
		virtual void onObjectChanged(const SWorldObject& object) = 0;
		virtual void onObjectRemoved(const std::string& name) = 0;
		virtual void onSelectedObjectChanged(const std::string& selected) = 0;
		virtual void onObjectsCleared() = 0;
	};

	struct ISimulationListener {
		virtual void onRunStateChanged(bool bRunning) = 0;
	};

	struct IErrorListener {
		virtual void onLoadScenarioFailed(const std::string& msg) = 0;
		virtual void onLoadURDFFailed(const std::string& msg) = 0;
		virtual void onLoadControllerFailed(const std::string& msg) = 0;
		virtual void onRunControllerFailed(const std::string& msg) = 0;
	};

	struct IPoseListener {
		virtual void onPoseAdded(std::string pose) = 0;
		virtual void onPoseRemoved(std::string pose) = 0;
		virtual void onPosesCleared() = 0;
		virtual void onPosesLoaded() = 0;
		virtual void onPoseRenamed(std::string oldName, std::string newName) = 0;
	};
	
	struct IScenarioInstance {
		virtual AF loadFromYAML(std::string path) = 0;
		virtual AF renameScenario(std::string newName) = 0;

		virtual AF setPostureService(std::string serviceName) = 0;
		virtual AF setJointStateTopic(std::string topicName) = 0;
		virtual AF setCommandTopic(std::string topicName) = 0;

		virtual AF makeURDFRelative(bool bRelative) = 0;
		virtual AF changeURDFPath(std::string& newPath) = 0;
		virtual const urdf::Model* getURDFModel() = 0; 

		virtual AF makeControllerRelative(bool bRelative) = 0;
		virtual AF changeControllerPath(std::string& newPath) = 0;
		virtual AF reloadController() = 0;
		virtual const giskard_core::QPController* getController() const = 0;

		virtual AF addEmptyPose() = 0;
		virtual AF addCurrentPose() = 0;
		virtual AF removePose(std::string name) = 0;
		virtual AF setPose(std::string name) = 0;
		virtual AF getPose(std::string name, SPose& out) = 0;
		virtual AF renamePose(std::string pose, std::string newName) = 0;

		virtual AF setJointPosition(std::string pose, std::string joint, double position) = 0;
		virtual AF renameJoint(std::string pose, std::string joint, std::string newName) = 0;
		virtual AF addJoint(std::string pose, std::string joint, double name) = 0;
		virtual AF removeJoint(std::string pose, std::string joint) = 0;

		virtual AF setSimTimestep(double dt) = 0;
		virtual AF setSimDefaultPose(std::string pose) = 0;
		virtual AF setSimUseTimestep(bool bUse) = 0;
		virtual AF setSimState(bool bRunning) = 0;
		virtual AF resetSim() = 0;

        virtual AF addSceneObject(SWorldObject* pObject) = 0;
		virtual AF removeSceneObject(std::string objectName) = 0;
		virtual AF selectSceneObject(std::string objectName) = 0;
		virtual AF attachSceneObject(std::string objectName, std::string frame, bool keepTransform) = 0;

        virtual AF setInputAssignment(boost::shared_ptr<IInputAssignment> assignment) = 0;

		virtual double getScalarObjectProperty(std::string object, std::string property) const = 0;
		virtual Eigen::Vector3d getVectorObjectProperty(std::string object, std::string property) const = 0;
		virtual Eigen::Affine3d getObjectTransform(std::string object, std::string target) const = 0;

		virtual const SScenarioContext* getContext() const = 0;
		
		// Listeners

		virtual void addURDFListener(IURDFListener* pList) = 0;
		virtual void removeURDFListener(IURDFListener* pList) = 0;
		
		virtual void addTopicListener(ITopicListener* pList) = 0;
		virtual void removeTopicListener(ITopicListener* pList) = 0;

		virtual void addControllerListener(IControllerListener* pList) = 0;
		virtual void removeControllerListener(IControllerListener* pList) = 0;

		virtual void addPoseListener(IPoseListener* pList) = 0;
		virtual void removePoseListener(IPoseListener* pList) = 0;

		virtual void addErrorListener(IErrorListener* pList) = 0;
		virtual void removeErrorListener(IErrorListener* pList) = 0;
		
		virtual void addScenarioListener(IScenarioListener* pList) = 0;
		virtual void removeScenarioListener(IScenarioListener* pList) = 0;
	};

	struct IScenarioReference {
		virtual void setScenario(IScenarioInstance* _pScenario) = 0;
	};
}
