cmake_minimum_required(VERSION 3.0.0)

project(ARSKDProjects)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# controls where cmake will install the files
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/install)

# define cmake module path so we can use Find
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/modules)

find_package(ARSDK REQUIRED)

## ARSDK
if(MSVC)
    # Add target for nvARPose
    add_library(nvARPose INTERFACE)
    target_include_directories(nvARPose INTERFACE ${ARSDK_INCLUDES_PATH})

    # Add target for NVCVImage
    add_library(NVCVImage INTERFACE)
    target_include_directories(NVCVImage INTERFACE ${ARSDK_INCLUDES_PATH})

endif()

add_definitions(-DNOMINMAX -DWIN32_LEAN_AND_MEAN)

# Set common build path for all targets
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# add subdirectory where to search for more CMakeLists files
add_subdirectory(projects)
