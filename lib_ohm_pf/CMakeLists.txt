cmake_minimum_required(VERSION 2.8.3)
project(lib_ohm_pf)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -DBENCHMARKING=0 -DTIMING=1 -Wall -std=c++11 -Wno-deprecated-declarations")

include_directories(
	include/
	include/distribute
	include/distribute/interfaces
)

link_directories(
    /usr/lib
)

add_library(OhmPf
    src/GaussianPdf.cpp
    src/Filter.cpp
    src/FilterOutputUpdater.cpp
    src/FilterUpdaterMeasurement.cpp
    src/FilterUpdaterTimed.cpp
    src/FilterUpdaterMeasurementOCS.cpp
    src/FilterUpdater.cpp
    src/LaserUpdater.cpp
    src/SampleSet.cpp
    src/LaserProbMapUpdater.cpp
    src/MapUpdater.cpp
    src/OdomUpdater.cpp
    src/PoseUpdater.cpp
    src/LVResampler.cpp
    src/FilterController.cpp
    src/OCSObserver.cpp
    src/STDResampler.cpp
    src/OCSClient.cpp
    src/DiffDriveUpdater.cpp
    src/ProbMap.cpp
    src/IMap.cpp
    src/Timer.cpp
    src/AdaptiveMean.cpp
    src/OmniDriveUpdater.cpp
    src/UtilitiesOhmPf.cpp
)

target_link_libraries(OhmPf 
	evocortex_core
)