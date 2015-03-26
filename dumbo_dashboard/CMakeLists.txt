cmake_minimum_required(VERSION 2.8.3)
project(dumbo_dashboard)

find_package(catkin REQUIRED COMPONENTS roscpp brics_actuator std_srvs std_msgs control_msgs)

find_package(Qt4 4.6 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})


include_directories(include 
  include/dumbo_dashboard
  ${catkin_INCLUDE_DIRS} 
  ${QT_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_BINARY_DIR})

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(CATKIN_DEPENDS roscpp brics_actuator std_srvs std_msgs control_msgs)

set(dumbo_dashboard_SOURCES src/main.cpp src/qnode.cpp src/mainwindow.cpp)
set(dumbo_dashboard_HEADERS include/dumbo_dashboard/qnode.hpp include/dumbo_dashboard/mainwindow.hpp)
set(dumbo_dashboard_FORMS ui/mainwindow.ui)
set(dumbo_dashboard_RESOURCES resources/images.qrc)

QT4_WRAP_CPP(dumbo_dashboard_HEADERS_MOC ${dumbo_dashboard_HEADERS})
QT4_WRAP_UI(dumbo_dashboard_FORMS_HEADERS ${dumbo_dashboard_FORMS})
QT4_ADD_RESOURCES(dumbo_dashboard_RESOURCES_RCC ${dumbo_dashboard_RESOURCES})


add_executable(dumbo_dashboard 
  ${dumbo_dashboard_SOURCES} 
  ${dumbo_dashboard_HEADERS_MOC}
  ${dumbo_dashboard_FORMS_HEADERS}
  ${dumbo_dashboard_RESOURCES_RCC})

target_link_libraries(dumbo_dashboard ${catkin_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(dumbo_dashboard cob_srvs_gencpp dumbo_srvs_gencpp brics_actuator_gencpp)

install(TARGETS dumbo_dashboard
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})