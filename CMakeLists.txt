cmake_minimum_required(VERSION 2.8.3)
project(csapex_system_information)

find_package(catkin REQUIRED COMPONENTS
    csapex)

find_package(Boost REQUIRED COMPONENTS regex)

catkin_package(
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

add_library(${PROJECT_NAME}
    ${sources}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
