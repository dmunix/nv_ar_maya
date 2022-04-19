project(AR_sample_body)

# setup source and header files
set(SOURCE_FILES src/BodyEngine.cpp  
    src/BodyTrack.cpp 
    ${ARSDK_UTILS_PATH}/RenderingUtils.cpp
)
set(HEADER_FILES  src/BodyEngine.h)

if(MSVC)
	set(SOURCE_FILES ${SOURCE_FILES} 
        ${ARSDK_INCLUDES_PATH}/../src/nvARProxy.cpp
        ${ARSDK_INCLUDES_PATH}/../src/nvCVImageProxy.cpp
    )

    set(HEADER_FILES ${HEADER_FILES} 
        ${ARSDK_UTILS_PATH}/RenderingUtils.h
    )
endif(MSVC)

# Set Visual Studio source filters
source_group("Source Files" FILES ${SOURCE_FILES})
source_group("Header Files" FILES ${HEADER_FILES})

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC ${ARSDK_INCLUDES_PATH})


# add ARSDK bin to the path
if(MSVC)
    target_link_libraries(${PROJECT_NAME} PUBLIC opencv346 utils_ARSDK)

    set(ARSDK_PATH_STR ${ARSDK_INCLUDES_PATH}/../../bin)
    set(OPENCV_PATH_STR ${CMAKE_CURRENT_SOURCE_DIR}/../external/opencv/bin)
    set(PATH_STR "PATH=%PATH%" ${ARSDK_PATH_STR})
    set(PATH_STR "PATH=%PATH%" ${OPENCV_PATH_STR})
    # set(CMD_ARG_STR "--model_path=\"${ARSDK_INCLUDES_PATH}/../../bin/models\"")
	# set_target_properties(${PROJECT_NAME} PROPERTIES 
	# 	FOLDER SampleApps 
	# 	VS_DEBUGGER_ENVIRONMENT "${PATH_STR}" 
	# 	VS_DEBUGGER_COMMAND_ARGUMENTS "${CMD_ARG_STR}"
	# )

endif(MSVC)
