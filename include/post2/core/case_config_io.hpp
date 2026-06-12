#pragma once

#include <string>

#include "post2/core/types.hpp"

namespace post2::core {

std::string case_config_to_json(const CaseConfig& config);
bool case_config_from_json(const std::string& text, CaseConfig* config, std::string* error);

bool load_case_config_file(const std::string& path, CaseConfig* config, std::string* error);
bool save_case_config_file(const std::string& path, const CaseConfig& config, std::string* error);

} // namespace post2::core
