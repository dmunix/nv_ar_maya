cmake_minimum_required(VERSION 3.0.0)


# ARSDK Utils
set(ARSDK_UTILS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/utils)
MESSAGE( STATUS "ARSDK_UTILS_PATH: " ${ARSDK_UTILS_PATH} )

# ARSDK Utils
add_library(utils_ARSDK INTERFACE)
target_include_directories(utils_ARSDK INTERFACE ${ARSDK_UTILS_PATH} ${OpenCV_INCLUDE_DIR})
target_link_libraries(utils_ARSDK INTERFACE GLM)

# add subdirectory where to search for more CMakeLists files
add_subdirectory(external)
add_subdirectory(AR_sample_body)
add_subdirectory(AR_maya_sample)
add_subdirectory(AR_maya_bodyTracking)
