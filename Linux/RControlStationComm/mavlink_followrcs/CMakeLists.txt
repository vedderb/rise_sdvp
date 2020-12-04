cmake_minimum_required(VERSION 2.8.12)

project(mavlink_followrcs)

find_package(Threads REQUIRED)

if(NOT MSVC)
    add_definitions("-std=c++11 -Wall -Wextra -Werror")
else()
    add_definitions("-std=c++11 -WX -W2")
endif()

find_package(MAVSDK REQUIRED)
find_package(Qt5 COMPONENTS Core Network REQUIRED)

add_executable(mavlink_followrcs
    follow_me.cpp
    ../rcontrolstationcomm.cpp
    rcs_location_provider.cpp
)

target_link_libraries(mavlink_followrcs
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_follow_me
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${CMAKE_THREAD_LIBS_INIT}
    Qt5::Core
    Qt5::Network
)
