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

    std_msgs::ColorRGBA rosColorRGBA(float r, float g, float b, float a) {
		std_msgs::ColorRGBA out;
        out.r = r;
        out.g = g;
        out.b = b;
		out.a = a;
		return out;
	}

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
