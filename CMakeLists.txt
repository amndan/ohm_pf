cmake_minimum_required ( VERSION 2.6 )

project ( ohmPf )

# # # #
# Includes
#

set ( INCLUDE_DIRS_OHMPF
	"${PROJECT_SOURCE_DIR}/include"
	#...
)

include_directories( ${INCLUDE_DIRS_OHMPF} )

# # # #
# libohmPf
#

set ( SOURCES_OHMPF
	${PROJECT_SOURCE_DIR}/src/Pose.cpp
	#...
)

add_library( ohmPf STATIC ${SOURCES_OHMPF})
target_include_directories ( ohmPf PUBLIC ${INCLUDE_DIRS} )

# # # #
# Testing
#

set ( TESTING_SOURCES_DIR
	${PROJECT_SOURCE_DIR}/testing/
)

add_executable( testPose ${TESTING_SOURCES_DIR}/testPose.cpp )
target_link_libraries( testPose ohmPf )









