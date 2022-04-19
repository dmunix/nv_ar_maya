#Maya finder module
#
# variables defined
#

# set the default maya version if not specified
if (NOT DEFINED MAYA_VERSION)
	set(MAYA_VERSION 2019 CACHE STRING "Maya version")
endif()

# OS specific environment setup
set(MAYA_LIB_SUFFIX "lib")
set(MAYA_INCLUDE_SUFFIX "include")
set(MAYA_COMPILE_DEFINITIONS "REQUIRE_IOSTREAM;_BOOL")
set(MAYA_TARGET_TYPE LIBRARY)

if (WIN32)
	#windows
	set(MAYA_INSTALL_BASE_DEFAULT "C:/Program Files/Autodesk")
	set(OPENMAYA_LIB OpenMaya.lib)
	set(MAYA_COMPILE_DEFINITIONS "${MAYA_COMPILE_DEFINITIONS};NT_PLUGIN")
	set(MAYA_PLUGIN_EXT ".mll")
	set(MAYA_TARGET_TYPE RUNTIME)
elseif(APPLE)
	#mac
	set(MAYA_INSTALL_BASE_DEFAULT "/Applications/Autodesk")
	set(OPENMAYA_LIB libOpenMaya.dylib)
	set(MAYA_LIB_SUFFIX "Maya.app/Contents/MacOS")
	set(MAYA_COMPILE_DEFINITIONS "${MAYA_COMPILE_DEFINITIONS};OSMac_")
	set(MAYA_PLUGIN_EXT ".bundle")
else()
	#linux
	set(MAYA_INSTALL_BASE_DEFAULT "/usr/autodesk")
	set(OPENMAYA_LIB libOpenMaya.so)
	set(MAYA_COMPILE_DEFINITIONS "${MAYA_COMPILE_DEFINITIONS};LINUX")
	set(MAYA_PLUGIN_EXT ".so")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()



set(MAYA_INSTALL_BASE_PATH ${MAYA_INSTALL_BASE_DEFAULT} CACHE STRING "Root Maya installation path")
set(MAYA_LOCATION ${MAYA_INSTALL_BASE_PATH}/maya${MAYA_VERSION})

# Maya library directory
find_path(MAYA_LIBRARY_DIR ${OPENMAYA_LIB}
	PATHS
		${MAYA_LOCATION}
		$ENV{MAYA_LOCATION}
	PATH_SUFFIXES
		"${MAYA_LIB_SUFFIX}/"
	DOC "Maya library path"
)

# Maya include directory
find_path(MAYA_INCLUDE_DIR maya/MFn.h
	PATHS
		${MAYA_LOCATION}
		$ENV{MAYA_LOCATION}
	PATH_SUFFIXES
		"${MAYA_INCLUDE_SUFFIX}/"
		DOC "Maya include path"
)

# Debug
MESSAGE( STATUS "MAYA_LIBRARY_DIR: " ${MAYA_LIBRARY_DIR} )
MESSAGE( STATUS "MAYA_INCLUDE_DIR: " ${MAYA_INCLUDE_DIR} )

# find Maya libraries
set(_MAYA_LIBRARIES OpenMaya OpenMayaAnim OpenMayaFX OpenMayaRender OpenMayaUI Foundation)
foreach(MAYA_LIB ${_MAYA_LIBRARIES})
	find_library(MAYA_${MAYA_LIB}_LIBRARY NAMES ${MAYA_LIB} PATHS ${MAYA_LIBRARY_DIR} NO_DEFAULT_PATH)
	set(MAYA_LIBRARIES ${MAYA_LIBRARIES} ${MAYA_${MAYA_LIB}_LIBRARY})
endforeach()


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Maya DEFAULT_MSG MAYA_INCLUDE_DIR MAYA_LIBRARIES)

# set the compiler target properties
function(MAYA_PLUGIN _target)
	if(WIN32)
		set_target_properties(${_target} PROPERTIES
		LINK_FLAGS "/export:initializePlugin /export:uninitializePlugin")
	endif()

	set_target_properties(${_target} PROPERTIES
		COMPILE_DEFINITIONS "${MAYA_COMPILE_DEFINITIONS}"
		PREFIX ""
		SUFFIX ${MAYA_PLUGIN_EXT}
	)
endfunction()
