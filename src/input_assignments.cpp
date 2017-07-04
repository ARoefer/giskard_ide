#include "giskard_sim/input_assignments.h"
#include "giskard_sim/utils.h"

namespace giskard_sim {
bool IInputAssignment::equals(const IInputAssignment& other) const { return getType() == other.getType() && name == other.name; }


bool ConstScalarAssignment::equals(const IInputAssignment& other) const {
	const ConstScalarAssignment* o = dynamic_cast<const ConstScalarAssignment*>(&other);
	return o && o->value == value && IInputAssignment::equals(other);
}

YAML::Node ConstScalarAssignment::toYAML() const {
	YAML::Node node;
	node["type"] = "constScalar";
	node["value"] = value;
	return node;
}

double ScalarPropertyAssignment::getValue() { 
	return scenario->getScalarObjectProperty(object, property);
}

bool ScalarPropertyAssignment::equals(const IInputAssignment& other) const {
	const ScalarPropertyAssignment* o = dynamic_cast<const ScalarPropertyAssignment*>(&other);
	return o && o->object == object && o->property == property && IInputAssignment::equals(other);
}

void ScalarPropertyAssignment::setScenario(IScenarioInstance* pScenario) {
	scenario = pScenario;
}

YAML::Node ScalarPropertyAssignment::toYAML() const {
	YAML::Node node;
	node["type"] = "scalarProperty";
	node["object"] = object;
	node["property"] = property;
	return node;
}

bool ConstVectorAssignment::equals(const IInputAssignment& other) const {
	const ConstVectorAssignment* o = dynamic_cast<const ConstVectorAssignment*>(&other);
	return o && o->value == value && IInputAssignment::equals(other);
}

YAML::Node ConstVectorAssignment::toYAML() const {
	YAML::Node node;
	node["type"] = "constVector";
	node["value"] = value;
	return node;
}

Eigen::Vector3d VectorPropertyAssignment::getValue() { 
	return scenario->getVectorObjectProperty(object, property);
}

bool VectorPropertyAssignment::equals(const IInputAssignment& other) const {
	const VectorPropertyAssignment* o = dynamic_cast<const VectorPropertyAssignment*>(&other);
	return o && o->object == object && o->property == property && IInputAssignment::equals(other);
}

void VectorPropertyAssignment::setScenario(IScenarioInstance* pScenario) {
	scenario = pScenario;
}

YAML::Node VectorPropertyAssignment::toYAML() const {
	YAML::Node node;
	node["type"] = "vectorProperty";
	node["object"] = object;
	node["property"] = property;
	return node;
}
Eigen::Vector3d VectorPositionAssignment::getValue() { 
	return scenario->getObjectTransform(object, target).translation();
}

bool VectorPositionAssignment::equals(const IInputAssignment& other) const {
	const VectorPositionAssignment* o = dynamic_cast<const VectorPositionAssignment*>(&other);
	return o && o->object == object && o->target == target && IInputAssignment::equals(other);
}

void VectorPositionAssignment::setScenario(IScenarioInstance* pScenario) {
	scenario = pScenario;
}

YAML::Node VectorPositionAssignment::toYAML() const {
	YAML::Node node;
	node["type"] = "positionQuery";
	node["object"] = object;
	node["target"] = target;
	return node;
}

bool ConstRotationAssignment::equals(const IInputAssignment& other) const {
	const ConstRotationAssignment* o = dynamic_cast<const ConstRotationAssignment*>(&other);
	return o && o->value == value && IInputAssignment::equals(other);
}

YAML::Node ConstRotationAssignment::toYAML() const {
	YAML::Node node;
	node["type"] = "constRotation";
	node["value"] = value;
	return node;
}

Eigen::Quaterniond RotationAssignment::getValue() { 
	return Eigen::Quaterniond(scenario->getObjectTransform(object, target).rotation());
}

bool RotationAssignment::equals(const IInputAssignment& other) const {
	const RotationAssignment* o = dynamic_cast<const RotationAssignment*>(&other);
	return o && o->object == object && o->target == target && IInputAssignment::equals(other);
}

void RotationAssignment::setScenario(IScenarioInstance* pScenario) {
	scenario = pScenario;
}

YAML::Node RotationAssignment::toYAML() const {
	YAML::Node node;
	node["type"] = "rotationQuery";
	node["object"] = object;
	node["target"] = target;
	return node;
}

bool ConstFrameAssignment::equals(const IInputAssignment& other) const {
	const ConstFrameAssignment* o = dynamic_cast<const ConstFrameAssignment*>(&other);
	return o && o->value == value && IInputAssignment::equals(other);
}

YAML::Node ConstFrameAssignment::toYAML() const {
	YAML::Node node;
	node["type"] = "constFrame";
	node["value"] = value;
	return node;
}

Eigen::Affine3d FrameAssignment::getValue() { 
	return scenario->getObjectTransform(object, target);
}

bool FrameAssignment::equals(const IInputAssignment& other) const {
	const FrameAssignment* o = dynamic_cast<const FrameAssignment*>(&other);
	return o && o->object == object && o->target == target && IInputAssignment::equals(other);
}

void FrameAssignment::setScenario(IScenarioInstance* pScenario) {
	scenario = pScenario;
}

YAML::Node FrameAssignment::toYAML() const {
	YAML::Node node;
	node["type"] = "frameQuery";
	node["object"] = object;
	node["target"] = target;
	return node;
}
}
