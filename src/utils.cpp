#include "giskard_sim/utils.h"

#include <ros/package.h>

#include <stdexcept>
using namespace std;

namespace giskard_sim {
	string resolvePath(const SFilePath& fp) {
		if (fp.packageRelative) {
			size_t sep = fp.path.find('/');
			string package = fp.path.substr(0, sep);
			string packPath = ros::package::getPath(package);
			return packPath + fp.path.substr(sep);
		}
		return fp.path;
	}

	string makePackageRelative(const string& path) {
		size_t lastSep = path.rfind('/');

		if (lastSep == string::npos || lastSep == 0)
			throw domain_error("Can't convert '" + path + "' to be package relative. Path must at least be one folder deep.");

		size_t sep = path.rfind('/', lastSep - 1);
        while (lastSep != 0) {
            string package = path.substr(sep + 1, lastSep - sep - 1);

			if (ros::package::getPath(package) != "") {
				return package + path.substr(lastSep);
			}

			lastSep = sep;
			sep = path.rfind('/', lastSep - 1);
		}

		throw domain_error("Can't convert '" + path + "' to be package relative. No part of the path matches a package name.");
	}

	Eigen::Quaterniond fromEulerRad(Eigen::Vector3d ang) {
        Eigen::Quaterniond out = Eigen::AngleAxisd(ang[0], Eigen::Vector3d::UnitX())
                      * Eigen::AngleAxisd(ang[1], Eigen::Vector3d::UnitY())
                      * Eigen::AngleAxisd(ang[2], Eigen::Vector3d::UnitZ());
        return out;
	}

	Eigen::Quaterniond fromEulerDeg(Eigen::Vector3d ang) {
		return fromEulerRad(ang * (M_PI / 180.0));
	}

	Eigen::Vector3d toEulerRad(Eigen::Quaterniond rot) {
        return toEulerRad(rot.toRotationMatrix());
	}

	Eigen::Vector3d toEulerDeg(Eigen::Quaterniond rot) {
		return toEulerDeg(rot.toRotationMatrix());
	}

    Eigen::Vector3d toEulerRad(Eigen::Matrix3d rot) {
        return rot.eulerAngles(0, 1, 2);
	}

    Eigen::Vector3d toEulerDeg(Eigen::Matrix3d rot) {
        return toEulerRad(rot) * (180.0 / M_PI);
	}

	Eigen::Affine3d makeAffine(Eigen::Vector3d position, Eigen::Quaterniond rotation) {
		return Eigen::Affine3d(Eigen::Translation3d(position)) * Eigen::Affine3d(rotation);
	}

    std_msgs::ColorRGBA rosColorRGBA(float r, float g, float b, float a) {
		std_msgs::ColorRGBA out;
        out.r = r;
        out.g = g;
        out.b = b;
		out.a = a;
		return out;
	}

    std_msgs::ColorRGBA rosColorRGBA(QColor color)  { return rosColorRGBA(color.redF(), color.greenF(), color.blueF(), color.alphaF()); }

	geometry_msgs::Point rosPoint(double x, double y, double z) {
		geometry_msgs::Point out;
		out.x = x;
		out.y = y;
		out.z = z;
		return out;
	}

	geometry_msgs::Vector3 rosVec3(double x, double y, double z) {
		geometry_msgs::Vector3 out;
		out.x = x;
		out.y = y;
		out.z = z;
		return out;
	}

	std_msgs::Header rosHeader(std::string frame, ros::Time stamp) {
		std_msgs::Header header;
		header.frame_id = frame;
		header.stamp = stamp;
		return header;
	}

}
