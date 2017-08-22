#include "giskard_sim/scenario_instance.h"
#include "giskard_sim/utils.h"
//#include "giskard_sim/input_assignments.h"

#include <iai_naive_kinematics_sim/SetJointState.h>

//#include <giskard_core/giskard_parser.hpp>
#include <tf_conversions/tf_eigen.h>

using namespace Eigen;
using namespace visualization_msgs;

namespace giskard_sim {

void add6DOFControls(InteractiveMarker& intMarker, bool bFixed = true, bool bVisible = false) {
	InteractiveMarkerControl control;

	if (bFixed) {
		control.orientation_mode = InteractiveMarkerControl::FIXED;
	}

	if (!bVisible)
		control.interaction_mode = InteractiveMarkerControl::NONE;	

	control.always_visible = true;

	control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
	if (bVisible)
    	control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    intMarker.controls.push_back(control);
    control.name = "move_x";
    if (bVisible)
		control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    intMarker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    if (bVisible)
		control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    intMarker.controls.push_back(control);
    control.name = "move_z";
    if (bVisible)
		control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    intMarker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    if (bVisible)
		control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    intMarker.controls.push_back(control);
    control.name = "move_y";
    if (bVisible)
		control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    intMarker.controls.push_back(control);
}

// INPORTANT FOR SINGELTON WORK AROUND
ScenarioInstance* ScenarioInstance::_currentInstance = 0;

ScenarioInstance::ScenarioInstance() 
: tfListener(nh)
, scenarioListeners()
, urdfListeners()
, controllerListeners()
, errorListeners()
, poseListeners()
, runner(cmdPublisher)
, intServer("giskard_marker_server", "", true)
{ 
    if(_currentInstance)
        ROS_ERROR("There should always only be one instance of giskard_sim::ScenarioInstance!");
	_currentInstance = this;

	cmdPublisher = nh.advertise<sensor_msgs::JointState>(context.cmdTopic, 1);
	jsSubscriber = nh.subscribe(context.jsTopic, 1, &ScenarioInstance::jointStateCB, this);
    updateTimer = nh.createTimer(ros::Duration(0.04), &ScenarioInstance::update, this);
    //updateTimer.stop();
    runner.setScenario(this);
}

ScenarioInstance::~ScenarioInstance() {
	_currentInstance = 0;
    ROS_INFO("ScenarioInstance deleted");
}

void ScenarioInstance::update(const ros::TimerEvent& event) {
	if (context.simSettings.bRunning && runner.isValid()) {
		AF result = runner.updateController(lastJointState);
        if(!result) {
        	notifyRunControllerFailed(result);
            setSimState(false);
        }
	}

	if (context.objects.size() > 0) {
		for (auto it = context.objects.begin(); it != context.objects.end(); it++) {
			tf::Transform tfTrans;
			tf::transformEigenToTF(it->second->transform, tfTrans);
			tfBroadcaster.sendTransform(tf::StampedTransform(tfTrans, ros::Time::now(), it->second->parent, it->second->name));
		}
	}
}

void ScenarioInstance::jointStateCB(const sensor_msgs::JointState& js) {
	lastJointState = js;
}

AF ScenarioInstance::loadFromYAML(string path) {
    context.poses.clear();
    notifyPosesCleared();
    notifyInputAssignmentsCleared();

    try {
		YAML::Node node = YAML::LoadFile(path);
		context = node.as<SScenarioContext>();
		for (auto it = context.inputAssignments.begin(); it != context.inputAssignments.end(); it++) {
			it->second->setScenario(this);
		}
		cmdPublisher = nh.advertise<sensor_msgs::JointState>(context.cmdTopic, 1);
		jsSubscriber = nh.subscribe(context.jsTopic, 1, &ScenarioInstance::jointStateCB, this);
	} catch (const YAML::Exception& e) {
        notifyLoadScenarioFailed("Error while loading scenario from '"+path+"':"+e.what());
		return AF(AF::Error, e.what());
	}

	if (!urdfModel.initFile(resolvePath(context.urdfPath))) {
		string errorMsg = "Can't load URDF referenced by "+context.urdfPath.toString();
		notifyLoadURDFFailed(errorMsg);
		return AF(AF::Error, errorMsg);
	}

	auto objects = context.objects;
	context.objects.clear();
	for (auto it = objects.begin(); it != objects.end(); it++) {
		addSceneObject(it->second.get());
	}

    setSimState(context.simSettings.bRunning);
	notifyTopicsChanged();
    notifyPosesLoaded();
    //notifyURDFLoaded();
    loadController();
    notifyInputAssignmentsLoaded();
    notifyScenarioLoaded(path);
	return AF();
}

AF ScenarioInstance::renameScenario(std::string newName) {
    if (newName.empty())
        return AF(AF::Failure, "Scenario name can't be empty.");

    context.name = newName;
    return AF();
}

AF ScenarioInstance::setPostureService(std::string serviceName) {
	context.setJSService = serviceName;
	notifyTopicsChanged();
	return AF();
}

AF ScenarioInstance::setJointStateTopic(std::string topicName) {
	context.jsTopic = topicName;
	jsSubscriber = nh.subscribe(context.jsTopic, 1, &ScenarioInstance::jointStateCB, this);
	notifyTopicsChanged();
	return AF();
}

AF ScenarioInstance::setCommandTopic(std::string topicName) {
	context.cmdTopic = topicName;
	cmdPublisher = nh.advertise<sensor_msgs::JointState>(context.cmdTopic, 1);
	notifyTopicsChanged();
	return AF();
}


AF ScenarioInstance::makePathRelative(SFilePath& path, bool bRelative) {
	if (!path.packageRelative && bRelative) {
		try {
			path.path = makePackageRelative(path.path);
			path.packageRelative = true;
		} catch (const domain_error& e) {
			return AF(AF::Error, e.what());
		}
	} else if (path.packageRelative && !bRelative) {
		path.path = resolvePath(path);
		path.packageRelative = false;
	}

	return AF();
}

AF ScenarioInstance::makeURDFRelative(bool bRelative) {
	return makePathRelative(context.urdfPath, bRelative);
}

AF ScenarioInstance::changeURDFPath(string& newPath) {
	string errorMsg = "Couldn't load URDF: "+newPath;
    if (context.urdfPath.packageRelative) {
        try {
            string path = makePackageRelative(newPath);
            if (path != context.urdfPath.path) {
                context.urdfPath.path = path;
                if(urdfModel.initFile(newPath)) {
                    notifyURDFLoaded();
                } else {
                    notifyLoadURDFFailed(errorMsg);
            		return AF(AF::Failure, errorMsg);
                }
            }
            return AF();
        } catch(domain_error &e) {
            context.urdfPath.packageRelative = false;
        }
    }

    if (newPath != context.urdfPath.path) {
        context.urdfPath.path = newPath;
        if(urdfModel.initFile(newPath)) {
            notifyURDFLoaded();
        } else {
            notifyLoadURDFFailed(errorMsg);
            return AF(AF::Failure, errorMsg);
        }
    }
    return AF();
}

AF ScenarioInstance::makeControllerRelative(bool bRelative) {
	return makePathRelative(context.controllerPath, bRelative);
}

AF ScenarioInstance::changeControllerPath(std::string& newPath) {
	string errorMsg = "Couldn't load Controller: "+newPath;
    if (context.controllerPath.packageRelative) {
        try {
            string path = makePackageRelative(newPath);
            if (path != context.controllerPath.path) {
                context.controllerPath.path = path;
            }
            return loadController();
        } catch(domain_error &e) {
            context.controllerPath.packageRelative = false;
        }
    }

    if (newPath != context.controllerPath.path) {
        context.controllerPath.path = newPath;
    }
    return loadController();
}

AF ScenarioInstance::loadController() {
	string path = resolvePath(context.controllerPath);
	giskard_core::QPControllerSpec spec;
    string msg = "Error while parsing '" + path + "':\n"; 
	
	if (path.find(".yaml") != string::npos) {
		try {
	        YAML::Node node = YAML::LoadFile(path);
	        spec = node.as<giskard_core::QPControllerSpec>();
    	} catch (const YAML::Exception& e) {
            notifyLoadControllerFailed(msg + e.what());
            return AF(AF::Failure, msg + e.what());
      	}
	} /* else {
 		try {
            std::ifstream t(path);
			std::string fileStr;

			t.seekg(0, std::ios::end);   
			fileStr.reserve(t.tellg());
			t.seekg(0, std::ios::beg);

			fileStr.assign((std::istreambuf_iterator<char>(t)),
			            std::istreambuf_iterator<char>());

			giskard_core::GiskardLangParser glParser;
			spec = glParser.parseQPController(fileStr);
		} catch (giskard_core::GiskardLangParser::EOSException e) {
            notifyLoadControllerFailed(msg + e.what());
			return AF(AF::Failure, msg + e.what()); 
		} catch (giskard_core::GiskardLangParser::ParseException e) {
            notifyLoadControllerFailed(msg + e.what());
			return AF(AF::Failure, msg + e.what());
		}             
	} */

	try {
		controller = giskard_core::generate(spec);
	} catch (std::exception e) {
        notifyLoadControllerFailed(msg + e.what());
		return AF(AF::Failure, msg + e.what()); 
	}

	notifyControllerLoaded();
	validateInputAssignments();
	return AF();
}


AF ScenarioInstance::addEmptyPose() {
    if (urdfModel.getName().empty())
        return AF(AF::Failure, "Can't add empty pose without valid URDF model.");

	string newPoseName = "new_pose";
	int counter = 1;
    while (context.poses.find(newPoseName + std::to_string(counter)) != context.poses.end())
		counter++;

	SPose newPose;
    newPose.name = newPoseName + std::to_string(counter);
    newPose.robotName = urdfModel.getName();

    context.poses[newPose.name] = newPose;
    notifyPoseAdded(newPose.name);
    return AF();
}

AF ScenarioInstance::addCurrentPose() {
    if (urdfModel.getName().empty())
        return AF(AF::Failure, "Can't add pose without valid URDF model.");

    boost::shared_ptr<const sensor_msgs::JointState> js = ros::topic::waitForMessage<sensor_msgs::JointState>(context.jsTopic);

    string newPoseName = "new_pose";
    int counter = 1;
    while (context.poses.find(newPoseName + std::to_string(counter)) != context.poses.end())
        counter++;

    SPose newPose;
    newPose.name = newPoseName + std::to_string(counter);
    newPose.robotName = urdfModel.getName();
    vector<string> joints = controller.get_input_names(giskard_core::tJoint);
    unordered_set<string> jointSet(joints.begin(), joints.end());
    for (size_t i = 0; i < js->name.size(); i++) {
        if (jointSet.find(js->name[i]) != jointSet.end()) {
            newPose.jointPos[js->name[i]] = js->position[i];
        }
    }

    context.poses[newPose.name] = newPose;

    notifyPoseAdded(newPose.name);
    return AF();
}

AF ScenarioInstance::removePose(string name) {
    context.poses.erase(name);
    notifyPoseRemoved(name);
    return AF();
}

AF ScenarioInstance::setPose(string name) {
    auto it = context.poses.find(name);
    if (it == context.poses.end())
        return AF(AF::Failure, "Can't set pose '" + name + "' because it is unknown.");

    iai_naive_kinematics_sim::SetJointState::Request req;
    iai_naive_kinematics_sim::SetJointState::Response res;
    req.state = it->second.toJointState();

    if (!ros::service::call(context.setJSService, req, res))
        return AF(AF::Error, "Couldn't call service '" + context.setJSService + "' to set joint state.");

    if (!res.success)
        return AF(AF::Failure, res.message);

    return AF();
}

AF ScenarioInstance::getPose(string name, SPose& out) {
	auto it = context.poses.find(name);
	if (it == context.poses.end())
		return AF(AF::Failure, "Pose '" + name + "' is unknown.");

	out = it->second;

	return AF();
}

AF ScenarioInstance::renamePose(string pose, string newName) {
	auto it = context.poses.find(pose);
	if (it == context.poses.end())
		return AF(AF::Failure, "Pose '" + pose + "' is unknown.");

	if (context.poses.find(newName) != context.poses.end())
		return AF(AF::Failure, "Can't rename pose. Name '" + newName + "' is already taken.");

	context.poses[newName] = context.poses[pose];
	context.poses[newName].name = newName;
	context.poses.erase(pose);

    notifyPoseRenamed(pose, newName);

	return AF();	
}

AF ScenarioInstance::setJointPosition(std::string pose, std::string joint, double position) {
    auto it = context.poses.find(pose);
	if (it == context.poses.end())
        return AF(AF::Failure, "Pose '" + pose + "' is unknown.");

    if (it->second.jointPos.find(joint) == it->second.jointPos.end())
        return AF(AF::Failure, "Joint '" + joint + "' is unknown.");

	it->second.jointPos[joint] = position;
	return AF();
}

AF ScenarioInstance::renameJoint(std::string pose, std::string joint, std::string newName) {
    auto it = context.poses.find(pose);
	if (it == context.poses.end())
        return AF(AF::Failure, "Pose '" + pose + "' is unknown.");

    if (it->second.jointPos.find(joint) != it->second.jointPos.end())
        return AF(AF::Failure, "Joint with '" + pose + "' already exists.");

	it->second.jointPos[newName] = it->second.jointPos[joint];
	it->second.jointPos.erase(joint);
	return AF();
}

AF ScenarioInstance::addJoint(std::string pose, std::string joint, double pos) {
    auto it = context.poses.find(pose);
	if (it == context.poses.end())
        return AF(AF::Failure, "Pose '" + pose + "' is unknown.");

    if (it->second.jointPos.find(joint) != it->second.jointPos.end())
        return AF(AF::Failure, "Joint with '" + joint+ "' already exists.");

    it->second.jointPos[joint] = pos;
	return AF();
}

AF ScenarioInstance::removeJoint(string pose, string joint) {
    auto it = context.poses.find(pose);
    if (it == context.poses.end())
        return AF(AF::Failure, "Pose '" + pose + "' is unknown.");

    it->second.jointPos.erase(joint);
    return AF();
}

AF ScenarioInstance::setSimTimestep(double dt) {
	if (dt <= 0.0)
		return AF(AF::Failure, "Timesteps of zero or less are not allowed!");

	context.simSettings.timeStep = dt;

	return AF();
}

AF ScenarioInstance::setSimDefaultPose(std::string pose) {
	auto it = context.poses.find(pose);
	if (it == context.poses.end())
		return AF(AF::Failure, "Can't set default pose to unknown pose '" + pose + "'");

	context.simSettings.defaultPose = pose;

	return AF();
}

AF ScenarioInstance::setSimUseTimestep(bool bUse) {
	context.simSettings.bUseTimeStep = bUse;
	return AF();
}

AF ScenarioInstance::setSimState(bool bRunning) {
	context.simSettings.bRunning = bRunning;
    // if (bRunning)
    //     updateTimer.start();
    // else
    //     updateTimer.stop();
	return AF();
}

AF ScenarioInstance::resetSim() {
	return setPose(context.simSettings.defaultPose);
}

Eigen::Affine3d ScenarioInstance::getObjectTransform(std::string object, std::string target) const {
	tf::StampedTransform temp;
	tfListener.waitForTransform(target, object, ros::Time(0), ros::Duration(0.5));
	tfListener.lookupTransform(target, object, ros::Time(0), temp);

	Eigen::Affine3d out = Eigen::Affine3d::Identity();
	tf::transformTFToEigen(temp, out);
	return out;
}

AF ScenarioInstance::addSceneObject(SWorldObject* pObject) {
	SWorldObject object = *pObject;

	object.visual.header = rosHeader("", ros::Time(0));
	
	auto it = context.objects.find(object.name);
	if (it == context.objects.end()) {
		InteractiveMarker intMarker;
		intMarker.header = rosHeader(object.parent, ros::Time(0));
		intMarker.name = object.name;
		tf::poseEigenToMsg(object.transform, intMarker.pose);

		InteractiveMarkerControl ctrl;
		ctrl.name = "visual";
		ctrl.always_visible = true;
		ctrl.markers.push_back(object.visual);
		ctrl.orientation.w = 1;
		ctrl.interaction_mode = InteractiveMarkerControl::MOVE_3D;
		intMarker.controls.push_back(ctrl);
		intMarker.scale = 1;
		add6DOFControls(intMarker, true);

		context.objects[object.name] = boost::shared_ptr<SWorldObject>(new SWorldObject(object));   

		intServer.insert(intMarker, &ScenarioInstance::_processInteractiveMarkerFeedback);
		// 'commit' changes and send to all clients
		intServer.applyChanges();
		notifyObjectAdded(object);
	} else {
		InteractiveMarker m;
		if (intServer.get(object.name, m)) {
			it->second = boost::shared_ptr<SWorldObject>(new SWorldObject(object));
			for(size_t i = 0; i < m.controls.size(); i++) {
				if (m.controls[i].name == "visual") {
					m.controls[i].markers[0] = object.visual;
					break;
				}
			}
			m.header.frame_id = object.parent;
			tf::poseEigenToMsg(object.transform, m.pose);
			intServer.insert(m, &ScenarioInstance::_processInteractiveMarkerFeedback);
			intServer.applyChanges();
			notifyObjectChanged(object);
		}
	}
    return AF();
}

AF ScenarioInstance::removeSceneObject(std::string objectName) {
	if (context.objects.find(objectName) != context.objects.end()) {
		context.objects.erase(objectName);
		intServer.erase(objectName);
		intServer.applyChanges();
		notifyObjectRemoved(objectName);
	}

	selectedObject = "";
	
	return AF();
}

AF ScenarioInstance::selectSceneObject(std::string objectName) {

	bool changed = false;
	if (!selectedObject.empty() && selectedObject != objectName) {
		InteractiveMarker m;
		if (intServer.get(selectedObject, m)) {
			for (size_t i = 0; i < m.controls.size(); i++) {
				InteractiveMarkerControl& c = m.controls[i];
				if (c.name.find("rotate") != std::string::npos 
				   || c.name.find("move") != std::string::npos)
					c.interaction_mode = InteractiveMarkerControl::NONE;
			}
			changed = true;
			intServer.insert(m, &ScenarioInstance::_processInteractiveMarkerFeedback);
		}
	}


	if (selectedObject != objectName && !objectName.empty()) {
		InteractiveMarker m;
		if (intServer.get(objectName, m)) {
			for (size_t i = 0; i < m.controls.size(); i++) {
				InteractiveMarkerControl& c = m.controls[i];
				if (c.name.find("rotate") != std::string::npos)
					c.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;

				if (c.name.find("move") != std::string::npos)
					c.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
			}
			changed = true;
			intServer.insert(m, &ScenarioInstance::_processInteractiveMarkerFeedback);
		} else {
			if (changed)
				intServer.applyChanges();
			return AF(AF::Failure, "Can't select non existant object '" + objectName + "'.");
		}	
	}

	if (changed) {
		selectedObject = objectName;
		intServer.applyChanges();
		notifySelectedObjectChanged(selectedObject);
	}
	
	return AF();
}

AF ScenarioInstance::attachSceneObject(std::string objectName, std::string frame, bool keepTransform) {
	auto it = context.objects.find(objectName);
	if (it != context.objects.end()) {
		SWorldObject updatedObject = *(it->second);
		if (!keepTransform) {
			Affine3d refT = getObjectTransform(updatedObject.parent, frame);
			updatedObject.transform = refT * updatedObject.transform;
		}
		updatedObject.parent = frame;
		addSceneObject(&updatedObject);

		return AF();
	}
	return AF(AF::Failure, "Unknown object '" + objectName + "'");
}


void ScenarioInstance::processInteractiveMarkerFeedback(const InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch (feedback->event_type) {
	case InteractiveMarkerFeedback::MOUSE_DOWN: {
		if (context.objects.find(feedback->marker_name) != context.objects.end() && selectedObject != feedback->marker_name) {
			selectSceneObject(feedback->marker_name);
		}
	}
	break;
	case InteractiveMarkerFeedback::POSE_UPDATE: {
		auto it = context.objects.find(feedback->marker_name);
		if (it != context.objects.end()) {
			tf::poseMsgToEigen(feedback->pose, it->second->transform);
			if (it->first != selectedObject)
				selectSceneObject(feedback->marker_name);
			notifyObjectChanged(*(it->second));
		}
	}
	break;
	default:
		break;
  }
}

void ScenarioInstance::_processInteractiveMarkerFeedback(const InteractiveMarkerFeedbackConstPtr &feedback) {
    if (ScenarioInstance::_currentInstance)
        ScenarioInstance::_currentInstance->processInteractiveMarkerFeedback(feedback);
}


AF ScenarioInstance::setInputAssignment(boost::shared_ptr<IInputAssignment> assignment) {
	try {
        const giskard_core::Scope::InputPtr& input = controller.get_scope().find_input(assignment->name);
		if (input->get_type() != assignment->getType())
			return AF(AF::Failure, "Mismatched type of input assignment for '" + input->name_ + "'");

		assignment->setScenario(this);
        auto it = context.inputAssignments.find(assignment->name);
        if (it == context.inputAssignments.end() || !it->second->equals(*assignment)) {
			context.inputAssignments[assignment->name] = assignment;
			notifyInputAssignmentChanged(assignment);
		}
	} catch (const std::exception& e) {
		return AF(AF::Failure, e.what());
	}
	return AF();
}

AF ScenarioInstance::validateInputAssignments() {
	auto inputMap = controller.get_input_map();

	{
	auto it = context.inputAssignments.begin();
	while (it != context.inputAssignments.end()) {
		auto fit = inputMap.find(it->first);
		if (fit == inputMap.end() || fit->second->get_type() != it->second->getType()) {
			notifyInputAssignmentDeleted(it->first);
            it = context.inputAssignments.erase(it);
		} else {
            inputMap.erase(it->first);
			it++;
		}
	}
	}

	for (auto it = inputMap.begin(); it != inputMap.end(); it++) {
		switch(it->second->get_type()) {
            case giskard_core::tScalar: {
                AssignmentPtr a(new ConstScalarAssignment(it->first));
				a->setScenario(this);
				context.inputAssignments[a->name] = a;
				notifyInputAssignmentChanged(a);
			}
			break;
            case giskard_core::tVector3: {
                AssignmentPtr a(new ConstVectorAssignment(it->first));
				a->setScenario(this);
				context.inputAssignments[a->name] = a;
				notifyInputAssignmentChanged(a);
			}
			break;
            case giskard_core::tRotation: {
                AssignmentPtr a(new ConstRotationAssignment(it->first));
				a->setScenario(this);
				context.inputAssignments[a->name] = a;
				notifyInputAssignmentChanged(a);
			}
			break;
            case giskard_core::tFrame: {
                AssignmentPtr a(new ConstFrameAssignment(it->first));
				a->setScenario(this);
				context.inputAssignments[a->name] = a;
				notifyInputAssignmentChanged(a);
			}
			break;
			default:
			break;
		}
	}

	return AF();
}

//////////////////////////// Listeners and Notifications /////////////////////////////////////////

void ScenarioInstance::notifyLoadScenarioFailed(string msg) {
	for(auto it = errorListeners.begin(); it != errorListeners.end(); it++)
		(*it)->onLoadScenarioFailed(msg);
}
void ScenarioInstance::notifyLoadURDFFailed(string msg) {
	for(auto it = errorListeners.begin(); it != errorListeners.end(); it++)
		(*it)->onLoadURDFFailed(msg);
    for(auto it = urdfListeners.begin(); it != urdfListeners.end(); it++)
        (*it)->onURDFChanged(0);
}
void ScenarioInstance::notifyLoadControllerFailed(string msg) {
	for(auto it = errorListeners.begin(); it != errorListeners.end(); it++)
		(*it)->onLoadControllerFailed(msg);
	for(auto it = controllerListeners.begin(); it != controllerListeners.end(); it++)
		(*it)->onControllerLoadFailed(msg);
}

void ScenarioInstance::notifyRunControllerFailed(string msg) {
	for(auto it = errorListeners.begin(); it != errorListeners.end(); it++)
		(*it)->onRunControllerFailed(msg);
}

void ScenarioInstance::notifyURDFLoaded() {
	for(auto it = urdfListeners.begin(); it != urdfListeners.end(); it++) {
		(*it)->onURDFChanged(&urdfModel);
	}
}
void ScenarioInstance::notifyControllerLoaded() {
	for(auto it = controllerListeners.begin(); it != controllerListeners.end(); it++) {
		(*it)->onControllerLoaded(&controller);
	}
}

void ScenarioInstance::notifyPoseAdded(string pose) {
	for(auto it = poseListeners.begin(); it != poseListeners.end(); it++) {
		(*it)->onPoseAdded(pose);
	}	
}

void ScenarioInstance::notifyPoseRemoved(string pose) {
	for(auto it = poseListeners.begin(); it != poseListeners.end(); it++) {
		(*it)->onPoseRemoved(pose);
	}
}

void ScenarioInstance::notifyPosesCleared() {
	for(auto it = poseListeners.begin(); it != poseListeners.end(); it++) {
		(*it)->onPosesCleared();
	}
}

void ScenarioInstance::notifyPosesLoaded() {
	for(auto it = poseListeners.begin(); it != poseListeners.end(); it++) {
		(*it)->onPosesLoaded();
	}
}

void ScenarioInstance::notifyPoseRenamed(string oldName, string newName) {
	for(auto it = poseListeners.begin(); it != poseListeners.end(); it++) {
		(*it)->onPoseRenamed(oldName, newName);
	}	
}

void ScenarioInstance::notifyScenarioLoaded(string path) {
	for(auto it = scenarioListeners.begin(); it != scenarioListeners.end(); it++) {
		(*it)->onScenarioLoaded(path, getContext());
	}	
}

void ScenarioInstance::notifyObjectAdded(const SWorldObject& object) {
	for(auto it = scenarioListeners.begin(); it != scenarioListeners.end(); it++) {
		(*it)->onObjectAdded(object);
	}
}

void ScenarioInstance::notifyObjectChanged(const SWorldObject& object) {
	for(auto it = scenarioListeners.begin(); it != scenarioListeners.end(); it++) {
		(*it)->onObjectChanged(object);
	}
}

void ScenarioInstance::notifyObjectRemoved(const std::string& name) {
	for(auto it = scenarioListeners.begin(); it != scenarioListeners.end(); it++) {
		(*it)->onObjectRemoved(name);
	}
}

void ScenarioInstance::notifySelectedObjectChanged(const std::string& selected) {
	for(auto it = scenarioListeners.begin(); it != scenarioListeners.end(); it++) {
		(*it)->onSelectedObjectChanged(selected);
	}
}

void ScenarioInstance::notifyTopicsChanged() {
	for(auto it = topicListeners.begin(); it != topicListeners.end(); it++) {
		(*it)->onTopicsChanged();
	}
}

void ScenarioInstance::addTopicListener(ITopicListener* pList) {
	if (pList)
		topicListeners.insert(pList);
}

void ScenarioInstance::removeTopicListener(ITopicListener* pList) {
	topicListeners.erase(pList);
}


void ScenarioInstance::addErrorListener(IErrorListener* pList) {
	if (pList)
		errorListeners.insert(pList);
}

void ScenarioInstance::removeErrorListener(IErrorListener* pList) {
	errorListeners.erase(pList);
}

void ScenarioInstance::ScenarioInstance::addURDFListener(IURDFListener* pList) {
	if (pList)
		urdfListeners.insert(pList);
}

void ScenarioInstance::ScenarioInstance::removeURDFListener(IURDFListener* pList) {
	urdfListeners.erase(pList);
}
	
void ScenarioInstance::ScenarioInstance::addControllerListener(IControllerListener* pList) {
	if (pList)
		controllerListeners.insert(pList);
}

void ScenarioInstance::ScenarioInstance::removeControllerListener(IControllerListener* pList) {
	controllerListeners.erase(pList);
}

void ScenarioInstance::addPoseListener(IPoseListener* pList) {
	if (pList)
		poseListeners.insert(pList);
}

void ScenarioInstance::removePoseListener(IPoseListener* pList) {
	poseListeners.erase(pList);
}

void ScenarioInstance::addScenarioListener(IScenarioListener* pList) {
	if (pList)
		scenarioListeners.insert(pList);
}

void ScenarioInstance::removeScenarioListener(IScenarioListener* pList) {
	scenarioListeners.erase(pList);
}

void ScenarioInstance::notifyInputAssignmentChanged(AssignmentPtr assignment) {
	for(auto it = controllerListeners.begin(); it != controllerListeners.end(); it++)
		(*it)->onInputAssignmentChanged(assignment);
}

void ScenarioInstance::notifyInputAssignmentDeleted(string name) {
	for(auto it = controllerListeners.begin(); it != controllerListeners.end(); it++)
		(*it)->onInputAssignmentDeleted(name);
}

void ScenarioInstance::notifyInputAssignmentsLoaded() {
	for(auto it = controllerListeners.begin(); it != controllerListeners.end(); it++)
		(*it)->onInputsLoaded(context.inputAssignments);
}

void ScenarioInstance::notifyInputAssignmentsCleared() {
	for(auto it = controllerListeners.begin(); it != controllerListeners.end(); it++)
		(*it)->onInputsCleared();
}

}
