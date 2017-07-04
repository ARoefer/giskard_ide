#pragma once

#include <giskard_core/scope.hpp>
#include <yaml-cpp/yaml.h>
#include "giskard_sim/interfaces.h"

using namespace std;

namespace giskard_sim {

struct IInputAssignment : public IScenarioReference {
	IInputAssignment(string _name) : name(_name) {}
	virtual giskard_core::InputType getType() const = 0;
	virtual YAML::Node toYAML() const = 0;
	const string name;
	virtual bool equals(const IInputAssignment& other) const;
};

typedef typename boost::shared_ptr<IInputAssignment> AssignmentPtr;

inline bool operator==(const IInputAssignment& a, const IInputAssignment& b) { return a.equals(b); }

struct IScalarAssignment : public IInputAssignment {
	IScalarAssignment(string name) : IInputAssignment(name) {}
	giskard_core::InputType getType() const {
		return giskard_core::tScalar;
    }

	virtual double getValue() = 0;
};

struct ConstScalarAssignment : public IScalarAssignment {
	ConstScalarAssignment(string name) : IScalarAssignment(name), value(0.0) {}
	ConstScalarAssignment(string name, double _value) : IScalarAssignment(name), value(_value) {}

	double getValue() { return value; }
	void setScenario(IScenarioInstance* pScenario) { }

	bool equals(const IInputAssignment& other) const;
	YAML::Node toYAML() const;
private:
	double value;
};

struct ScalarPropertyAssignment : public IScalarAssignment {
	ScalarPropertyAssignment(string name, string _object, string _property)
	: IScalarAssignment(name)
	, scenario(0)
	, object(_object)
	, property(_property) {}

	ScalarPropertyAssignment(IScenarioInstance* pScenario, string name, string _object, string _property) 
	: IScalarAssignment(name)
	, scenario(pScenario)
	, object(_object)
	, property(_property) {}

	double getValue();
	bool equals(const IInputAssignment& other) const;
	void setScenario(IScenarioInstance* pScenario);
	YAML::Node toYAML() const;
	const string object, property;
private:
	IScenarioInstance* scenario;
};

struct IVectorAssignment : public IInputAssignment {
	IVectorAssignment(string name) : IInputAssignment(name) {}
	giskard_core::InputType getType() const {
		return giskard_core::tVector3;
	};

	virtual Eigen::Vector3d getValue() = 0;
};

struct ConstVectorAssignment : public IVectorAssignment {
	ConstVectorAssignment(string name) : IVectorAssignment(name), value(0,0,0) {}
	ConstVectorAssignment(string name, Eigen::Vector3d _value) : IVectorAssignment(name), value(_value) {}

	Eigen::Vector3d getValue() { return value; }
	void setScenario(IScenarioInstance* pScenario) { }
	bool equals(const IInputAssignment& other) const;
	YAML::Node toYAML() const;
private:
	Eigen::Vector3d value;
};

struct VectorPropertyAssignment : public IVectorAssignment {
	VectorPropertyAssignment(string name, string _object, string _property)
	: IVectorAssignment(name)
	, scenario(0)
	, object(_object)
	, property(_property) {}

	VectorPropertyAssignment(IScenarioInstance* pScenario, string name, string _object, string _property) 
	: IVectorAssignment(name)
	, scenario(pScenario)
	, object(_object)
	, property(_property) {}

	Eigen::Vector3d getValue();
	bool equals(const IInputAssignment& other) const;
	void setScenario(IScenarioInstance* pScenario);
	YAML::Node toYAML() const;
	const string object, property;
private:
	IScenarioInstance* scenario;
};

struct VectorPositionAssignment : public IVectorAssignment {
	VectorPositionAssignment(string name, string _object, string _target)
	: IVectorAssignment(name)
	, scenario(0)
	, object(_object)
	, target(_target) {}

	VectorPositionAssignment(IScenarioInstance* pScenario, string name, string _object, string _target) 
	: IVectorAssignment(name)
	, scenario(pScenario)
	, object(_object)
	, target(_target) {}

	Eigen::Vector3d getValue();
	bool equals(const IInputAssignment& other) const;
	void setScenario(IScenarioInstance* pScenario);
	YAML::Node toYAML() const;
	const string object, target;
private:
	IScenarioInstance* scenario;
};

struct IRotationAssignment : public IInputAssignment {
	IRotationAssignment(string name) : IInputAssignment(name) {}
	giskard_core::InputType getType() const {
		return giskard_core::tRotation;
	}

	virtual Eigen::Quaterniond getValue() = 0;
};

struct ConstRotationAssignment : public IRotationAssignment {
	ConstRotationAssignment(string name) : IRotationAssignment(name), value(Eigen::Quaterniond::Identity()) {}
	ConstRotationAssignment(string name, Eigen::Quaterniond _value) : IRotationAssignment(name), value(_value) {}

	Eigen::Quaterniond getValue() { return value; }
	void setScenario(IScenarioInstance* pScenario) { }
	bool equals(const IInputAssignment& other) const;
	YAML::Node toYAML() const;
private:
	Eigen::Quaterniond value;
};

struct RotationAssignment : public IRotationAssignment {
	RotationAssignment(string name, string _object, string _target)
	: IRotationAssignment(name)
	, scenario(0)
	, object(_object)
	, target(_target) {}

	RotationAssignment(IScenarioInstance* pScenario, string name, string _object, string _target) 
	: IRotationAssignment(name)
	, scenario(pScenario)
	, object(_object)
	, target(_target) {}

	Eigen::Quaterniond getValue();
	bool equals(const IInputAssignment& other) const;
	void setScenario(IScenarioInstance* pScenario);
	YAML::Node toYAML() const;
	const string object, target;
private:
	IScenarioInstance* scenario;
};

struct IFrameAssignment : public IInputAssignment {
	IFrameAssignment(string name) : IInputAssignment(name) {}
	giskard_core::InputType getType() const {
		return giskard_core::tFrame;
	}

	virtual Eigen::Affine3d getValue() = 0;
};

struct ConstFrameAssignment : public IFrameAssignment {
	ConstFrameAssignment(string name) : IFrameAssignment(name), value(Eigen::Affine3d::Identity()) {}
	ConstFrameAssignment(string name, Eigen::Affine3d _value) : IFrameAssignment(name), value(_value) {}

	Eigen::Affine3d getValue() { return value; }
	void setScenario(IScenarioInstance* pScenario) { }

	bool equals(const IInputAssignment& other) const;
	YAML::Node toYAML() const;
private:
	Eigen::Affine3d value;
};

struct FrameAssignment : public IFrameAssignment {
	FrameAssignment(string name, string _object, string _target)
	: IFrameAssignment(name)
	, scenario(0)
	, object(_object)
	, target(_target) {}

	FrameAssignment(IScenarioInstance* pScenario, string name, string _object, string _target) 
	: IFrameAssignment(name)
	, scenario(pScenario)
	, object(_object)
	, target(_target) {}

	Eigen::Affine3d getValue();
	bool equals(const IInputAssignment& other) const;
	void setScenario(IScenarioInstance* pScenario);
	YAML::Node toYAML() const;
	const string object, target;
private:
	IScenarioInstance* scenario;
};

}
