find_package(OIS REQUIRED)

if(NOT OIS_FOUND)
	message(SEND_ERROR "Failed to find OIS.")
else(NOT OIS_FOUND)

	set(SRCS
		./src/BaseApplication.cpp
		./src/SampleApp.cpp
		./src/DotSceneLoader.cpp
		./include/BaseApplication.h
		./include/SampleApp.h
		./include/DotSceneLoader.h
		./include/rapidxml.hpp
	)

	set(SRCS ${SRCS})

	if(WIN32)
		getenv_path(OGRE_BYATIS)
		set(OGRE_SAMPLES_INCLUDEPATH
			${ENV_OGRE_BYATIS}/Samples/include
		)
	endif(WIN32)

	if(UNIX)
		find_path(OGRE_SAMPLES_INCLUDEPATH SdkTrays.h
			/usr/share/OGRE/Samples/Common/include/
			/usr/local/share/OGRE/Samples/Common/include/
		)
	endif(UNIX)

	include_directories(./include ${OIS_INCLUDE_DIRS}
		${OGRE_INCLUDE_DIRS}
		${OGRE_Terrain_INCLUDE_DIR}
		${Boost_INCLUDE_DIR}
		${OGRE_SAMPLES_INCLUDEPATH}
        ${DEPENDENCIES_INCLUDES}
	)

	if(WIN32)
		file(GLOB SampleLoadApp_res ./res/*.rc)
	endif(WIN32)

	if(NOT APPLE)
		add_executable(SampleLoadApp WIN32 ${SRCS} ${SampleLoadApp_res})
	else()
		add_executable(SampleLoadApp MACOSX_BUNDLE 
			${SRCS} 
			${SampleLoadApp_res})
	endif(NOT APPLE)

	set_target_properties(SampleLoadApp PROPERTIES DEBUG_POSTFIX _d)

	if(APPLE)
		find_package(IOKIT REQUIRED)
		find_package(Carbon REQUIRED)
        TARGET_LINK_LIBRARIES(SampleLoadApp ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${IOKit_LIBRARIES} ${Carbon_LIBRARIES} ${OGRE_Terrain_LIBRARY} PagedGeometry)
    ELSE()
        TARGET_LINK_LIBRARIES(SampleLoadApp ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${OGRE_Terrain_LIBRARY} PagedGeometry ${Boost_LIBRARIES})
	endif(APPLE)

	if(APPLE)
		install(TARGETS SampleLoadApp BUNDLE DESTINATION ${PREFIX}bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
		install(TARGETS SampleLoadApp BUNDLE DESTINATION ${PREFIX}Debug CONFIGURATIONS Debug)
	endif()

	if(WIN32)
		add_custom_command(TARGET SampleLoadApp PRE_BUILD 
			COMMAND if not exist ..\\RunPath\\bin mkdir ..\\RunPath\\bin)
		add_custom_command(TARGET SampleLoadApp POST_BUILD 
			COMMAND copy \"$(TargetPath)\" ..\\RunPath\\bin)

		install(FILES ${OGRE_PLUGIN_DIR_REL}/OIS.dll
			DESTINATION bin
			CONFIGURATIONS Release)
		install(FILES ${OGRE_PLUGIN_DIR_DBG}/OIS_d.dll
			DESTINATION bin
			CONFIGURATIONS Debug)
		install(TARGETS SampleLoadApp
			RUNTIME DESTINATION bin
			CONFIGURATIONS Release)
	endif(WIN32)

endif(NOT OIS_FOUND)

# vim: set sw=2 ts=2 noet:
