#Nvidia AR SDK finder module

# OS specific environment setup
if (MSVC)
	#windows
	set(ARSDK_INSTALL_BASE_DIR "D:/dev/rez/external/NVIDIA_AR_SDK/0.7.6.2")
	set(ARSDK_INCLUDES_PATH ${ARSDK_INSTALL_BASE_DIR}/nvar/include)
	MESSAGE( STATUS "ARSDK_INCLUDES_PATH: " ${ARSDK_INCLUDES_PATH} )
endif()
