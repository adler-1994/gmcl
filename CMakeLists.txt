cmake_minimum_required(VERSION 3.1)
project(gmcl)

include(CheckIncludeFile)
include(CheckSymbolExists)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin REQUIRED
  COMPONENTS
    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    message_filters
    nav_msgs
    rosbag
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_msgs
    tf2_ros
)

find_package(Boost REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
   cfg/GMCL.cfg
)

catkin_package(
  CATKIN_DEPENDS
    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    rosbag
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_msgs
    tf2_ros
  INCLUDE_DIRS include
  LIBRARIES gmcl_sensors gmcl_map gmcl_pf
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(src/include)

check_include_file(unistd.h HAVE_UNISTD_H)
if (HAVE_UNISTD_H)
  add_definitions(-DHAVE_UNISTD_H)
endif (HAVE_UNISTD_H)

check_symbol_exists(drand48 stdlib.h HAVE_DRAND48)
if (HAVE_DRAND48)
  add_definitions(-DHAVE_DRAND48)
endif (HAVE_DRAND48)

add_library(gmcl_pf
                    src/gmcl/pf/pf.c
                    src/gmcl/pf/pf_kdtree.c
                    src/gmcl/pf/pf_pdf.c
                    src/gmcl/pf/pf_vector.c
                    src/gmcl/pf/eig3.c
                    src/gmcl/pf/pf_draw.c)

add_library(gmcl_map
                    src/gmcl/map/map.c
                    src/gmcl/map/map_cspace.cpp
                    src/gmcl/map/map_espace.cpp
                    src/gmcl/map/map_range.c
                    src/gmcl/map/map_store.c
                    src/gmcl/map/map_draw.c)

add_library(gmcl_sensors
                    src/gmcl/sensors/gmcl_sensor.cpp
                    src/gmcl/sensors/gmcl_odom.cpp
                    src/gmcl/sensors/gmcl_laser.cpp)
target_link_libraries(gmcl_sensors gmcl_map gmcl_pf)


add_executable(gmcl
                       src/gmcl_node.cpp)
add_dependencies(gmcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gmcl
    gmcl_sensors gmcl_map gmcl_pf
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

install(TARGETS
    gmcl
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    gmcl_sensors gmcl_map gmcl_pf
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/gmcl/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


