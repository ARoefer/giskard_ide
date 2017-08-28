#pragma once

#include <giskard_core/specifications.hpp>


namespace giskard_sim {
	struct IGiskardParser {
        virtual std::string getName() const = 0;
        virtual std::string getFileSuffix() const = 0;
		virtual giskard_core::QPControllerSpec loadFromFile(const std::string& path) = 0;
    };
}