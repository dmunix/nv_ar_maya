project(AR_maya_sample)

find_package(Maya REQUIRED)

# specify include and lib directories
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )
include_directories(${MAYA_INCLUDE_DIR})
link_directories(${MAYA_LIBRARY_DIR})

MESSAGE ( STATUS "Include Dir : " ${CMAKE_CURRENT_SOURCE_DIR}/include )

# setup source and header files
set(SOURCE_FILES 
    src/socket.cpp
    src/mayaConnection.cpp
    src/main.cpp
)
set(HEADER_FILES  
    src/socket.h 
    src/mayaConnection.h 
)


if(MSVC)
  set(CMAKE_CXX_FLAGS_RELEASE "/MT")
  set(CMAKE_CXX_FLAGS_DEBUG "/MTd")
endif(MSVC)

# Set Visual Studio source filters
source_group("Source Files" FILES ${SOURCE_FILES})
source_group("Header Files" FILES ${HEADER_FILES})


# specify we need a shared library file
add_executable( ${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES} )


# and which libraries to link with when compiling
# target_link_libraries(${PROJECT_NAME} )
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC ${ARSDK_INCLUDES_PATH})


# add ARSDK and OPENCV bin to the path
if(MSVC)
    target_link_libraries(${PROJECT_NAME} PUBLIC opencv346 utils_ARSDK ${MAYA_LIBRARIES})

    set(ARSDK_PATH_STR ${ARSDK_INCLUDES_PATH}/../../bin)
    set(OPENCV_PATH_STR ${CMAKE_CURRENT_SOURCE_DIR}/../external/opencv/bin)
    set(PATH_STR "PATH=%PATH%" ${ARSDK_PATH_STR})
    set(PATH_STR "PATH=%PATH%" ${OPENCV_PATH_STR})

endif(MSVC)
