#pragma once

#include <giskard/scope.hpp>
#include <yaml-cpp/yaml.h>
#include "giskard_sim/interfaces.h"

using namespace std;

namespace giskard_sim {

struct IInputAssignment : public IScenarioReference {
		virtual giskard::Scope::InputTypes getType() const = 0;
		virtual YAML::Node toYAML() const = 0;
};

struct IScalarAssignment : public IInputAssignment {
	giskard::Scope::InputTypes getType() const {
		return giskard::Scope::Scalar;
        }

	virtual double getValue() = 0;
};

struct ConstScalarAssignment : public IScalarAssignment {
	ConstScalarAssignment(double _value) : value(_value) {}

	double getValue() { return value; }
	void setScenario(IScenarioInstance* pScenario) { }

	YAML::Node toYAML() const {
		YAML::Node node;
		node["type"] = "constScalar";
		node["value"] = value;
                return node;
	}
private:
	double value;
};

struct ScalarPropertyAssignment : public IScalarAssignment {
	ScalarPropertyAssignment(string _object, string _property)
	: scenario(0)
	, object(_object)
	, property(_property) {}

	ScalarPropertyAssignment(IScenarioInstance* pScenario, string _object, string _property) 
	: scenario(pScenario)
	, object(_object)
	, property(_property) {}

	double getValue() { 
                return scenario->getScalarObjectProperty(object, property);
	}

	void setScenario(IScenarioInstance* pScenario) {
		scenario = pScenario;
	}

	YAML::Node toYAML() const {
		YAML::Node node;
		node["type"] = "scalarProperty";
		node["object"] = object;
		node["property"] = property;
                return node;
	}
private:
	IScenarioInstance* scenario;
	string object, property;
};

struct IVectorAssignment : public IInputAssignment {
	giskard::Scope::InputTypes getType() const {
		return giskard::Scope::Vector;
	};

	virtual Eigen::Vector3d getValue() = 0;
};

struct ConstVectorAssignment : public IVectorAssignment {
	ConstVectorAssignment(Eigen::Vector3d _value) : value(_value) {}

	Eigen::Vector3d getValue() { return value; }
	void setScenario(IScenarioInstance* pScenario) { }

	YAML::Node toYAML() const {
		YAML::Node node;
		node["type"] = "constVector";
		node["value"] = value;
                return node;
	}
private:
	Eigen::Vector3d value;
};

struct VectorPropertyAssignment : public IVectorAssignment {
	VectorPropertyAssignment(string _object, string _property)
	: scenario(0)
	, object(_object)
	, property(_property) {}

	VectorPropertyAssignment(IScenarioInstance* pScenario, string _object, string _property) 
	: scenario(pScenario)
	, object(_object)
	, property(_property) {}

	Eigen::Vector3d getValue() { 
                return scenario->getVectorObjectProperty(object, property);
	}

	void setScenario(IScenarioInstance* pScenario) {
		scenario = pScenario;
	}

	YAML::Node toYAML() const {
		YAML::Node node;
		node["type"] = "vectorProperty";
		node["object"] = object;
		node["property"] = property;
                return node;
	}
private:
	IScenarioInstance* scenario;
	string object, property;
};

struct VectorPositionAssignment : public IVectorAssignment {
	VectorPositionAssignment(string _object, string _target)
	: scenario(0)
	, object(_object)
	, target(_target) {}

	VectorPositionAssignment(IScenarioInstance* pScenario, string _object, string _target) 
	: scenario(pScenario)
	, object(_object)
	, target(_target) {}

	Eigen::Vector3d getValue() { 
		return scenario->getObjectTransform(object, target).translation();
	}

	void setScenario(IScenarioInstance* pScenario) {
		scenario = pScenario;
	}

	YAML::Node toYAML() const {
		YAML::Node node;
		node["type"] = "positionQuery";
		node["object"] = object;
		node["target"] = target;
                return node;
	}
private:
	IScenarioInstance* scenario;
	string object, target;
};

struct IRotationAssignment : public IInputAssignment {
	giskard::Scope::InputTypes getType() const {
		return giskard::Scope::Rotation;
	}

	virtual Eigen::Quaterniond getValue() = 0;
};

struct RotationAssignment : public IRotationAssignment {
	RotationAssignment(string _object, string _target)
	: scenario(0)
	, object(_object)
	, target(_target) {}

	RotationAssignment(IScenarioInstance* pScenario, string _object, string _target) 
	: scenario(pScenario)
	, object(_object)
	, target(_target) {}

	Eigen::Quaterniond getValue() { 
                return Eigen::Quaterniond(scenario->getObjectTransform(object, target).rotation());
	}

	void setScenario(IScenarioInstance* pScenario) {
		scenario = pScenario;
	}

	YAML::Node toYAML() const {
		YAML::Node node;
		node["type"] = "rotationQuery";
		node["object"] = object;
		node["target"] = target;
                return node;
	}
private:
	IScenarioInstance* scenario;
	string object, target;
};

struct IFrameAssignment : public IInputAssignment {
	giskard::Scope::InputTypes getType() const {
		return giskard::Scope::Frame;
	}

	virtual Eigen::Affine3d getValue() = 0;
};

struct FrameAssignment : public IFrameAssignment {
	FrameAssignment(string _object, string _target)
	: scenario(0)
	, object(_object)
	, target(_target) {}

	FrameAssignment(IScenarioInstance* pScenario, string _object, string _target) 
	: scenario(pScenario)
	, object(_object)
	, target(_target) {}

	Eigen::Affine3d getValue() { 
		return scenario->getObjectTransform(object, target);
	}

	void setScenario(IScenarioInstance* pScenario) {
		scenario = pScenario;
	}

	YAML::Node toYAML() const {
		YAML::Node node;
		node["type"] = "frameQuery";
		node["object"] = object;
		node["target"] = target;
                return node;
	}
private:
	IScenarioInstance* scenario;
	string object, target;
};

}
