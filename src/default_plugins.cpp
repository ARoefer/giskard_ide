#include "giskard_sim/plugin_interfaces.h"

#include <giskard_core/yaml_parser.hpp>

#include <string>

namespace giskard_sim {
class StdGiskardYAMLParser : public IGiskardParser {
public:
    std::string getName() const { return "Giskard YAML parser"; }
    std::string getFileSuffix() const { return "yaml"; }
    giskard_core::QPControllerSpec loadFromFile(const std::string& path) {
        YAML::Node node = YAML::LoadFile(path);
        return node.as<giskard_core::QPControllerSpec>();
    }
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(giskard_sim::StdGiskardYAMLParser, giskard_sim::IGiskardParser)