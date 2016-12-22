#-------------------------------------------------
#
# Project created by QtCreator 2016-08-24T11:14:11
#
#-------------------------------------------------

QT += serialport
QT += network
QT       -= core gui

TARGET = module_vehicle_MAVLINK
TEMPLATE = lib

DEFINES += MODULE_VEHICLE_MAVLINK_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += module_vehicle_mavlink.cpp \
    Vehicles/Ardupilot/ardupilot_attitude.cpp \
    Vehicles/Ardupilot/ardupilot_flightmode.cpp \
    Vehicles/Ardupilot/ardupilot_global_position.cpp \
    Vehicles/Ardupilot/ardupilot_gps_status.cpp \
    Vehicles/Ardupilot/ardupilot_local_position.cpp \
    Vehicles/Ardupilot/ardupilot_mission.cpp \
    Vehicles/Ardupilot/ardupilot_position.cpp \
    Vehicles/Ardupilot/ardupilot_status.cpp \
    Vehicles/Ardupilot/data_ardupilot.cpp \
    Vehicles/Ardupilot/ardupilot_home_position.cpp

HEADERS += module_vehicle_mavlink.h\
        module_vehicle_mavlink_global.h \
    generic_message_definition_mavlink.h \
    Vehicles/Ardupilot/ardupilot_attitude.h \
    Vehicles/Ardupilot/ardupilot_flightmode.h \
    Vehicles/Ardupilot/ardupilot_global_position.h \
    Vehicles/Ardupilot/ardupilot_gps_status.h \
    Vehicles/Ardupilot/ardupilot_local_position.h \
    Vehicles/Ardupilot/ardupilot_mission.h \
    Vehicles/Ardupilot/ardupilot_position.h \
    Vehicles/Ardupilot/ardupilot_status.h \
    Vehicles/Ardupilot/data_ardupilot.h \
    Vehicles/Ardupilot/data_ardupilot_global.h \
    Vehicles/Ardupilot/ardupilot_home_position.h

INCLUDEPATH += $$PWD/../../mavlink_cpp/V2/ardupilotmega

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/module_vehicle_MAVLINK.lib release/module_vehicle_MAVLINK.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/module_vehicle_MAVLINK.lib debug/module_vehicle_MAVLINK.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/module_vehicle_MAVLINK
headers.files   += \
        module_vehicle_mavlink.h \
        module_vehicle_mavlink_global.h \
        generic_message_definition_mavlink.h
INSTALLS       += headers

headers_Vehicle_Ardupilot.path    = $$(MACE_ROOT)/include/module_vehicle_MAVLINK/Vehicles/Ardupilot
headers_Vehicle_Ardupilot.files   += Vehicles/Ardupilot/ardupilot_attitude.h \
    Vehicles/Ardupilot/ardupilot_flightmode.h \
    Vehicles/Ardupilot/ardupilot_global_position.h \
    Vehicles/Ardupilot/ardupilot_gps_status.h \
    Vehicles/Ardupilot/ardupilot_local_position.h \
    Vehicles/Ardupilot/ardupilot_mission.h \
    Vehicles/Ardupilot/ardupilot_position.h \
    Vehicles/Ardupilot/ardupilot_status.h \
    Vehicles/Ardupilot/data_ardupilot.h \
    Vehicles/Ardupilot/data_ardupilot_global.h
INSTALLS       += headers_Vehicle_Ardupilot

INCLUDEPATH += $$(MACE_ROOT)/include

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../comms/release/ -lcomms
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../comms/debug/ -lcomms
else:unix: LIBS += -L$$OUT_PWD/../comms/ -lcomms

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata


unix{
    EigenInclude = $$system(pkg-config --cflags eigen3)
    EigenInclude = $$replace(EigenInclude, "-I", "")/eigen3
    INCLUDEPATH += $$EigenInclude
}
win32{
    INCLUDEPATH += "C:\Program Files (x86)\Eigen\include\eigen3"
}



