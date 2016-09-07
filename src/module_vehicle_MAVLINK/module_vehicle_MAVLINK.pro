#-------------------------------------------------
#
# Project created by QtCreator 2016-08-24T11:14:11
#
#-------------------------------------------------

QT += serialport
QT       -= core gui

TARGET = module_vehicle_MAVLINK
TEMPLATE = lib

DEFINES += MODULE_VEHICLE_MAVLINK_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += module_vehicle_mavlink.cpp

HEADERS += module_vehicle_mavlink.h\
        module_vehicle_mavlink_global.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}



win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../comms/release/ -lcomms
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../comms/debug/ -lcomms
else:unix: LIBS += -L$$OUT_PWD/../comms/ -lcomms


INCLUDEPATH += $$PWD/../../mavlink_cpp/V2/ardupilotmega

INCLUDEPATH += $$PWD/../
DEPENDPATH += $$PWD/../

