#pragma once

#include "rviz/properties/editable_combo_box.h"

namespace giskard_sim {

        class PkgFileComboBox : public rviz::EditableComboBox {
        Q_OBJECT
        public:
                PkgFileComboBox(QWidget* parent = 0);
                void setPackage(std::string package);
                void addSuffix(std::string suffix);
        private:
                void refreshOptions();
                void findFiles(std::string folder);

                std::vector<std::string> options;
                std::string package;
                std::string pkgPath;
                std::vector<std::string> filters; 
        };
}
