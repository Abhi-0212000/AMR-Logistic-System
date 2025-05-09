// Copyright 2025 Abhishek Nannuri
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "amr_navigation_system/utils/common_utils.hpp"

#include <pwd.h>  // Include this header for getpwuid

#include <cstdlib>
#include <stdexcept>

namespace fs = std::filesystem;

namespace amr_navigation
{

bool checkFileExists(const std::string & filePath)
{
  return fs::exists(filePath) && fs::is_regular_file(filePath);
}

void createDirectory(const std::string & dirPath)
{
  if (!fs::exists(dirPath)) {
    if (!fs::create_directories(dirPath)) {
      throw std::runtime_error("Failed to create directory: " + dirPath);
    }
  }
}

std::string expandTilde(const std::string & path)
{
  if (!path.empty() && path[0] == '~') {
    const char * home = getenv("HOME");
    if (!home) {
      home = getpwuid(getuid())->pw_dir;  // NOLINT(runtime/threadsafe_fn)
    }
    return std::string(home) + path.substr(1);
  }
  return path;
}

std::string getDefaultDebugPath()
{
  std::string basePath;

  // Try to use XDG_DATA_HOME environment variable
  const char * xdgDataHome = std::getenv("XDG_DATA_HOME");
  if (xdgDataHome && *xdgDataHome) {
    basePath = xdgDataHome;
  } else {
    // Fall back to HOME environment variable
    const char * home = std::getenv("HOME");
    if (home && *home) {
      basePath = std::string(home) + "/.local/share";
    } else {
      // If HOME is not available, use current directory
      basePath = fs::current_path().string();
    }
  }

  return basePath + "AMR/graph_debug";
}

}  // namespace amr_navigation
