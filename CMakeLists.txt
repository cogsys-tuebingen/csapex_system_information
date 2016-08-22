cmake_minimum_required(VERSION 2.8.3)
project(csapex_system_information)

find_package(catkin REQUIRED COMPONENTS
    csapex
    csapex_core_plugins)

find_package(Boost REQUIRED COMPONENTS regex)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES csapex_system_information
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

file(GLOB sources
    src/*.hpp
    src/*.cpp
    src/*.h
)

add_library(csapex_system_information
    ${sources}
)

target_link_libraries(csapex_system_information
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS csapex_system_information
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
