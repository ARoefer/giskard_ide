#include "giskard_sim/pkg_file_combo_box.h"
#include <algorithm> // for std::sort
#include <ros/package.h>
#include <dirent.h>

namespace giskard_sim {
    PkgFileComboBox::PkgFileComboBox(QWidget* parent)
    : rviz::EditableComboBox(parent)
	{ }

    void PkgFileComboBox::setPackage(std::string _package) {
		if (package != _package) {
			package = _package;
			refreshOptions();
		}
	}

	void PkgFileComboBox::addSuffix(std::string suffix) {
		if (std::find(filters.begin(), filters.end(), suffix) == filters.end()) {
			filters.push_back(suffix);
			refreshOptions();
		}
	}

	void PkgFileComboBox::findFiles(std::string folder) {
		DIR *dir;
		struct dirent *ent;
		if ((dir = opendir(folder.c_str())) != NULL) {
			std::string shortPath = folder.substr(pkgPath.size());
			while ((ent = readdir(dir)) != NULL) {
				std::string fileName = ent->d_name;
				if (ent->d_type == DT_DIR && fileName != "." && fileName != "..") {
					findFiles(folder + '/' + ent->d_name);
				} else if (ent->d_type == DT_REG) {
					for (size_t i = 0; i < filters.size(); i++) {
						if (fileName.size() >= filters[i].size() 
						 && fileName.find(filters[i], fileName.size() - filters[i].size()) != std::string::npos) {
							options.push_back((shortPath + '/' + fileName));
						 	break;
						 }
					}
				}
			}
			closedir(dir);
		}
	}

	void PkgFileComboBox::refreshOptions() {
		options.clear();
		
		pkgPath = ros::package::getPath(package);
		if (!pkgPath.empty()) {
			findFiles(pkgPath);
		}
		
		clear();
		std::sort( options.begin(), options.end() );

		for( size_t i = 0; i < options.size(); i++ ) {
			addItem( QString::fromStdString(options[ i ]));
		}			
	}
}
