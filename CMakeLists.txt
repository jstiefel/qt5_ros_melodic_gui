cmake_minimum_required(VERSION 2.8.3)
project(qt5_ros_melodic_gui)

add_definitions(-std=c++11 -fPIC -Wall -Wextra)

##########
# CATKIN #
##########
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

##################
# QT ENVIRONMENT #
##################
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)

# non-ROS:
find_package(Qt5 COMPONENTS Core Gui Widgets)

############
# SECTIONS #
############

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    )

###########
# SOURCES #
###########

set(QT_SOURCES
    src/main.cpp
    src/mainwindow.cpp
    src/qnode.cpp

    include/nav_ui/mainwindow.hpp
    include/nav_ui/qnode.hpp
    )

#############
# LIBRARIES #
#############

#cs_add_library(${PROJECT_NAME}
#
#        )

############
# BINARIES #
############
cs_add_executable(nav_ui_node ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(nav_ui_node
#    ${PROJECT_NAME}
    Qt5::Core
    Qt5::Widgets
)

##########
# EXPORT #
##########
cs_install()
cs_export()