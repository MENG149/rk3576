#pragma once

#include <string>

namespace rkfast {

bool BindCurrentThread(const std::string& cpu_spec, std::string* error = nullptr);

}  // namespace rkfast
