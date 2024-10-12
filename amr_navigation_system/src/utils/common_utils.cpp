#include "amr_navigation_system/utils/common_utils.hpp"
#include <stdexcept>
#include <cstdlib>

namespace fs = std::filesystem;

namespace amr_navigation {

bool checkFileExists(const std::string& filePath) {
    return fs::exists(filePath) && fs::is_regular_file(filePath);
}

void createDirectory(const std::string& dirPath) {
    if (!fs::exists(dirPath)) {
        if (!fs::create_directories(dirPath)) {
            throw std::runtime_error("Failed to create directory: " + dirPath);
        }
    }
}

std::string getDefaultDebugPath() {
    std::string basePath;
    
    // Try to use XDG_DATA_HOME environment variable
    const char* xdgDataHome = std::getenv("XDG_DATA_HOME");
    if (xdgDataHome && *xdgDataHome) {
        basePath = xdgDataHome;
    } else {
        // Fall back to HOME environment variable
        const char* home = std::getenv("HOME");
        if (home && *home) {
            basePath = std::string(home) + "/.local/share";
        } else {
            // If HOME is not available, use current directory
            basePath = fs::current_path().string();
        }
    }
    
    return basePath + "/amr_navigation_system/debug";
}

}  // namespace amr_navigation