cmake_minimum_required(VERSION 2.6)
enable_testing()
project(planningToCalibrate CXX C)

# Mac ONLY. Define Relative Path on Mac OS
if(NOT DEFINED CMAKE_MACOSX_RPATH)
  set(CMAKE_MACOSX_RPATH 0)
endif()


# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})
message("GTSAM Libraries: ${GTSAM_LIBRARIES}")

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)
include(GtsamMatlabWrap)


# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Boost
find_package(Boost 1.50 REQUIRED)
find_package(Boost COMPONENTS filesystem system thread serialization REQUIRED)
include_directories(${Boost_INCLUDE_DIR})


# include current source folder, at the very beginning
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR})

# Include factors
file(GLOB srcs "src/*.cpp" "src/*.h")
include_directories(src)
add_library (planningToCalibrate SHARED ${srcs} ${GTSAM_LIBRARIES})
target_link_libraries(planningToCalibrate ${Boost_LIBRARIES} gtsam)

# Compile wrapper
wrap_and_install_library(planningToCalibrate.h ${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}" "")

# Unit tests
add_subdirectory(tests)

# Install config and export files
#GtsamMakeConfigFile(gpmp2)
#export(TARGETS ${gpmp2_EXPORTED_TARGETS} FILE gpmp2-exports.cmake)

