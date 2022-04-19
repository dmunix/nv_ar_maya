# Eigen finder module
#
# variables defined
#

# OS specific environment setup
set(EIGEN_INCLUDE_SUFFIX "include")

if (WIN32)
	#windows
	set(EIGEN_INSTALL_BASE_DEFAULT "C:/Program Files (x86)")
endif()



set(EIGEN_INSTALL_BASE_PATH ${EIGEN_INSTALL_BASE_DEFAULT} CACHE STRING "Root Eigen installation path")
set(EIGEN_LOCATION ${EIGEN_INSTALL_BASE_PATH}/Eigen3)

# Maya include directory
find_path(EIGEN_INCLUDE_DIR Eigen/src/Core/Array.h
	PATHS
		${EIGEN_LOCATION}
		$ENV{EIGEN_LOCATION}
	PATH_SUFFIXES
		"${EIGEN_INCLUDE_SUFFIX}/eigen3/"
		DOC "Eigen include path"
)

# Debug
MESSAGE( STATUS "EIGEN_INCLUDE_DIR: " ${EIGEN_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen DEFAULT_MSG EIGEN_INCLUDE_DIR)
