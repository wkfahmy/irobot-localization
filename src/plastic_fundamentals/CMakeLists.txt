# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(plastic_fundamentals)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs create_fundamentals message_generation)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_STANDARD 17)

#uncomment if you have defined messages
add_message_files(FILES
    Cell.msg
    Grid.msg
    Pose.msg
    Row.msg

    Line.msg
    Point.msg

        AbsEncoder.msg
)
#uncomment if you have defined services
add_service_files(FILES
    ExecutePlan.srv
    PublishMarker.srv
    Move.srv
    moveToPosition.srv
)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)

#include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES std_msgs create_fundamentals
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS python-serial
    CATKIN_DEPENDS roscpp rospy sensor_msgs create_fundamentals message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(align src/align.cpp)
target_link_libraries(align ${catkin_LIBRARIES})
add_dependencies(align create_fundamentals_generate_messages_cpp)

add_executable(mcl_localization src/mcl_localization.cpp)
target_link_libraries(mcl_localization ${catkin_LIBRARIES})
add_dependencies(mcl_localization create_fundamentals_generate_messages_cpp)

add_executable(marker_relay src/marker_relay.cpp)
target_link_libraries(marker_relay ${catkin_LIBRARIES})
add_dependencies(marker_relay create_fundamentals_generate_messages_cpp)

add_executable(marker_test src/marker_test.cpp)
target_link_libraries(marker_test ${catkin_LIBRARIES})
add_dependencies(marker_test create_fundamentals_generate_messages_cpp)

add_executable(simulator src/simulator.cpp)
target_link_libraries(simulator ${catkin_LIBRARIES})
add_dependencies(simulator create_fundamentals_generate_messages_cpp)

add_executable(mcl_wanderer src/mcl_wanderer.cpp)
target_link_libraries(mcl_wanderer ${catkin_LIBRARIES})
add_dependencies(mcl_wanderer create_fundamentals_generate_messages_cpp)

add_executable(calibrate src/calibrate.cpp)
target_link_libraries(calibrate ${catkin_LIBRARIES})
add_dependencies(calibrate create_fundamentals_generate_messages_cpp)

add_executable(play_guile src/play_guile.cpp)
target_link_libraries(play_guile ${catkin_LIBRARIES})
add_dependencies(play_guile ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(move src/move.cpp)
target_link_libraries(move ${catkin_LIBRARIES})
add_dependencies(move ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(graph_utils src/graph_utils.cpp)
target_link_libraries(graph_utils ${catkin_LIBRARIES})
add_dependencies(graph_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(move_to_position src/move_to_position.cpp)
target_link_libraries(move_to_position
    graph_utils              
    ${catkin_LIBRARIES}
)
add_dependencies(move_to_position
    graph_utils              
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)





