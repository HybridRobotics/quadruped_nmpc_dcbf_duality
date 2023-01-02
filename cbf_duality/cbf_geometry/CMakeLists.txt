cmake_minimum_required(VERSION 3.10)
project(cbf_geometry)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        qpoases_catkin
        ocs2_core
        )

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        qpoases_catkin
        ocs2_core
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
        src/ConvexRegion2d.cpp
        src/DistanceQp2d.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}_test
            test/ConvexRegion2dTest.cpp
            )

    target_link_libraries(${PROJECT_NAME}_test
            ${PROJECT_NAME}
            gtest_main
            )
endif (CATKIN_ENABLE_TESTING)
