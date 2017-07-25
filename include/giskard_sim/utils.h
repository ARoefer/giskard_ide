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
	inline std::string toRGBACSS(std_msgs::ColorRGBA color) { 
		return "rgba(" + std::to_string(color.r * 255) + ',' 
		               + std::to_string(color.g * 255) + ',' 
					   + std::to_string(color.b * 255) + ',' 
					   + std::to_string(color.a * 255) + ')'; 
	}

	// Other ROS data types
	geometry_msgs::Point rosPoint(double x, double y, double z);
    inline geometry_msgs::Point rosPoint(Eigen::Vector3d vec) { return rosPoint(vec[0], vec[1], vec[2]); }

	geometry_msgs::Vector3 rosVec3(double x, double y, double z);
    inline geometry_msgs::Vector3 rosVec3(Eigen::Vector3d vec) { return rosVec3(vec[0], vec[1], vec[2]); }

	std_msgs::Header rosHeader(std::string frame, ros::Time stamp);
}

// ----------- Encoding and decoding ROS messages using yaml-cpp ---------
namespace YAML {

	
}
