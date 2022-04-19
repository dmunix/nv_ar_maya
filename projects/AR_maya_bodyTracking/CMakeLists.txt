project(AR_maya_bodyTracking)

find_package(Maya REQUIRED)

# specify include and lib directories
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )
include_directories(${MAYA_INCLUDE_DIR})
link_directories(${MAYA_LIBRARY_DIR})

# setup source and header files
set(SOURCE_FILES 
    src/BodyEngine.cpp 
    src/BodyTracking.cpp 
    src/socket.cpp
    src/mayaConnection.cpp
    src/main.cpp

    # ${ARSDK_UTILS_PATH}/RenderingUtils.cpp
)
set(HEADER_FILES  
    src/BodyEngine.h 
    src/BodyTracking.h 
    src/socket.h 
    src/mayaConnection.h 
)

if(MSVC)
	set(SOURCE_FILES ${SOURCE_FILES} 
        ${ARSDK_INCLUDES_PATH}/../src/nvARProxy.cpp
        ${ARSDK_INCLUDES_PATH}/../src/nvCVImageProxy.cpp
    )

    set(HEADER_FILES ${HEADER_FILES} 
        ${ARSDK_UTILS_PATH}/RenderingUtils.h
    )
endif(MSVC)


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
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC ${ARSDK_INCLUDES_PATH})


# set the plugin directory after compilation
# install(TARGETS ${PROJECT_NAME} ${MAYA_TARGET_TYPE} DESTINATION plugins)


# add ARSDK and OPENCV bin to the path
if(MSVC)
    # target .libs
    target_link_libraries(${PROJECT_NAME} PUBLIC opencv346 utils_ARSDK ${MAYA_LIBRARIES})
    # MAKE SURE THE BIN FOLDERS ARE ADDED TO PATH IN WINDOWS
    
    # Set the PATH for the DEBUGGER
    set(ARSDK_PATH_STR ${ARSDK_INCLUDES_PATH}/../../bin)
    set(MAYA_PATH_STR ${MAYA_LOCATION}/bin)
    set(OPENCV_PATH_STR ${CMAKE_CURRENT_SOURCE_DIR}/../external/opencv/bin)
    set(PATH_STR "PATH=%PATH%" ${ARSDK_PATH_STR})
    set(PATH_STR "PATH=%PATH%" ${MAYA_PATH_STR})
    set(PATH_STR "PATH=%PATH%" ${OPENCV_PATH_STR})

    set(CMD_ARG_STR "--model_path=\"${ARSDK_INCLUDES_PATH}/../../bin/models\"")
	set_target_properties(${PROJECT_NAME} PROPERTIES 
		FOLDER SampleApps 
		VS_DEBUGGER_ENVIRONMENT "${PATH_STR}" 
		VS_DEBUGGER_COMMAND_ARGUMENTS "${CMD_ARG_STR}"
	)

    
endif(MSVC)
