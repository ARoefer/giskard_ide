#pragma once

#include "giskard_sim/datamodel.h"

#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <QColor>

using namespace std;

namespace giskard_sim {
	string resolvePath(const SFilePath& fp);
	string makePackageRelative(const string& path);

	Eigen::Quaterniond fromEulerRad(Eigen::Vector3d ang);
	Eigen::Quaterniond fromEulerDeg(Eigen::Vector3d ang);
    Eigen::Vector3d toEulerRad(Eigen::Matrix3d rot);
	Eigen::Vector3d toEulerRad(Eigen::Quaterniond rot);
    Eigen::Vector3d toEulerDeg(Eigen::Matrix3d rot);
    Eigen::Vector3d toEulerDeg(Eigen::Quaterniond rot);
	Eigen::Affine3d makeAffine(Eigen::Vector3d position, Eigen::Quaterniond rotation);

	inline bool operator==(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) { return  a.isApprox(b); }
	inline bool operator==(const Eigen::Affine3d& a, const Eigen::Affine3d& b)       { return  a.isApprox(b); }
	inline bool operator!=(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) { return !a.isApprox(b); }
	inline bool operator!=(const Eigen::Affine3d& a, const Eigen::Affine3d& b)       { return !a.isApprox(b); }

	// Convenience methods
	// ROS Colors
	std_msgs::ColorRGBA rosColorRGBA(float r = 1.f, float g = 1.f, float b = 1.f, float a = 1.f);
    std_msgs::ColorRGBA rosColorRGBA(QColor color);
    inline QColor toQColor(std_msgs::ColorRGBA color) { return QColor::fromRgbF(color.r,color.g,color.b,color.a); }
	inline std_msgs::ColorRGBA rosColorRed(float a = 1.f)    { return giskard_sim::rosColorRGBA(0.9, 0.0, 0.0, a); }
	inline std_msgs::ColorRGBA rosColorGreen(float a = 1.f)  { return giskard_sim::rosColorRGBA(0.0, 0.9, 0.0, a); }
	inline std_msgs::ColorRGBA rosColorBlue(float a = 1.f)   { return giskard_sim::rosColorRGBA(0.0, 0.0, 0.9, a); }
	inline std_msgs::ColorRGBA rosColorCyan(float a = 1.f)   { return giskard_sim::rosColorRGBA(0.0, 0.9, 0.9, a); }
	inline std_msgs::ColorRGBA rosColorPurple(float a = 1.f) { return giskard_sim::rosColorRGBA(0.9, 0.0, 0.9, a); }
	inline std_msgs::ColorRGBA rosColorYellow(float a = 1.f) { return giskard_sim::rosColorRGBA(0.9, 0.9, 0.0, a); }
	inline std_msgs::ColorRGBA rosColorWhite(float a = 1.f)  { return giskard_sim::rosColorRGBA(1.0, 1.0, 1.0, a); }
	inline std_msgs::ColorRGBA rosColorBlack(float a = 1.f)  { return giskard_sim::rosColorRGBA(0.0, 0.0, 0.0, a); }
	inline std_msgs::ColorRGBA rosColorGrey(float a = 1.f)   { return giskard_sim::rosColorRGBA(0.5, 0.5, 0.5, a); }

	// Other ROS data types
	geometry_msgs::Point rosPoint(double x, double y, double z);
    inline geometry_msgs::Point rosPoint(Eigen::Vector3d vec) { return rosPoint(vec[0], vec[1], vec[2]); }

	geometry_msgs::Vector3 rosVec3(double x, double y, double z);
    inline geometry_msgs::Vector3 rosVec3(Eigen::Vector3d vec) { return rosVec3(vec[0], vec[1], vec[2]); }

	std_msgs::Header rosHeader(std::string frame, ros::Time stamp);
}

// ----------- Encoding and decoding ROS messages using yaml-cpp ---------
namespace YAML {

	// CAREFUL: Timestamp is always set to NOW
	template<>
	struct convert<std_msgs::Header>
	{
		static Node encode(const std_msgs::Header& h) {
			Node node;
			node["frame_id"] = h.frame_id;
			return node;
		}

		static bool decode(const Node& node, std_msgs::Header& h) {
			try {
				h.stamp = ros::Time::now();
				h.frame_id = node["frame_id"].as<string>();
				return true;
			} catch (const YAML::Exception& e) {
				return false;
			}
		}
	};

	template<>
	struct convert<std_msgs::ColorRGBA>
	{
		static Node encode(const std_msgs::ColorRGBA& c) {
			Node node;
			node["r"] = c.r;
			node["g"] = c.g;
			node["b"] = c.b;
			node["a"] = c.a;
			return node;
		}

		static bool decode(const Node& node, std_msgs::ColorRGBA& c) {
			try {
				c.r = node["r"].as<double>();
				c.g = node["g"].as<double>();
				c.b = node["b"].as<double>();
				c.a = node["a"].as<double>();
				return true;
			} catch (const YAML::Exception& e) {
				return false;
			}
		}
	};

	template<>
	struct convert<geometry_msgs::Point>
	{
		static Node encode(const geometry_msgs::Point& p) {
			Node node;
			node["x"] = p.x;
			node["y"] = p.y;
			node["z"] = p.z;
			return node;
		}

		static bool decode(const Node& node, geometry_msgs::Point& p) {
			try {
				p.x = node["x"].as<double>();
				p.y = node["y"].as<double>();
				p.z = node["z"].as<double>();
				return true;
			} catch (const YAML::Exception& e) {
				return false;
			}
		}
	};

	template<>
	struct convert<geometry_msgs::Vector3>
	{
		static Node encode(const geometry_msgs::Vector3& v) {
			Node node;
			node["x"] = v.x;
			node["y"] = v.y;
			node["z"] = v.z;
			return node;
		}

		static bool decode(const Node& node, geometry_msgs::Vector3& v) {
			try {
				v.x = node["x"].as<double>();
				v.y = node["y"].as<double>();
				v.z = node["z"].as<double>();
				return true;
			} catch (const YAML::Exception& e) {
				return false;
			}
		}
	};

	template<>
	struct convert<geometry_msgs::Quaternion>
	{
		static Node encode(const geometry_msgs::Quaternion& q) {
			Node node;
			node["x"] = q.x;
			node["y"] = q.y;
			node["z"] = q.z;
			node["w"] = q.w;
			return node;
		}

		static bool decode(const Node& node, geometry_msgs::Quaternion& q) {
			try {
				q.x = node["x"].as<double>();
				q.y = node["y"].as<double>();
				q.z = node["z"].as<double>();
				q.w = node["w"].as<double>();
				return true;
			} catch (const YAML::Exception& e) {
				return false;
			}
		}
	};

	template<>
	struct convert<geometry_msgs::Pose>
	{
		static Node encode(const geometry_msgs::Pose& p) {
			Node node;
			node["position"] = p.position;
			node["orientation"] = p.orientation;
			return node;
		}

		static bool decode(const Node& node, geometry_msgs::Pose& p) {
			try {
				p.position = node["position"].as<geometry_msgs::Point>();
				p.orientation = node["orientation"].as<geometry_msgs::Quaternion>();
				return true;
			} catch (const YAML::Exception& e) {
				return false;
			}
		}
	};

	template<>
	struct convert<visualization_msgs::Marker>
	{
		static Node encode(const visualization_msgs::Marker& m) {
			Node node;
			node["header"] = m.header;
			node["ns"] = m.ns;
			node["id"] = m.id;
			node["type"] = m.type;
			node["action"] = m.action;
			node["pose"] = m.pose;
			node["scale"] = m.scale;
			node["color"] = m.color;
			node["points"] = m.points;
			node["colors"] = m.colors;
			node["text"] = m.text;
			node["mesh_resource"] = m.mesh_resource;
			node["mesh_use_embedded_materials"] = m.mesh_use_embedded_materials;
			return node;
		}

		static bool decode(const Node& node, visualization_msgs::Marker& m) {
			try {
				m.header        = node["header"].as<std_msgs::Header>();
				m.ns            = node["ns"].as<string>();
				m.id            = node["id"].as<int>();
				m.type          = node["type"].as<int>();
				m.action        = node["action"].as<int>();
				m.pose          = node["pose"].as<geometry_msgs::Pose>();
				m.scale         = node["scale"].as<geometry_msgs::Vector3>();
				m.color         = node["color"].as<std_msgs::ColorRGBA>();
				m.points        = node["points"].as<vector<geometry_msgs::Point>>();
				m.colors        = node["colors"].as<vector<std_msgs::ColorRGBA>>();
				m.text          = node["text"].as<string>();
				m.mesh_resource = node["mesh_resource"].as<string>();
				m.mesh_use_embedded_materials = node["mesh_use_embedded_materials"].as<bool>();
				return true;
			} catch (const YAML::Exception& e) {
				return false;
			}
		}
	};
}
