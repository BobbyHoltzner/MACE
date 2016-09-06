#-------------------------------------------------
#
# Project created by QtCreator 2016-08-31T13:55:42
#
#-------------------------------------------------
QT += serialport
QT       -= core gui

TARGET = comms
TEMPLATE = lib

DEFINES += COMMS_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += \
    serial_link.cpp \
    mavlink_protocol.cpp \
    link_marshaler.cpp \
    serial_configuration.cpp

HEADERS +=\
    i_link.h \
    serial_link.h \
    i_protocol.h \
    comms_global.h \
    mavlink_protocol.h \
    link_marshaler.h \
    i_mavlink_protocol_events.h \
    i_link_events.h \
    serial_configuration.h \
    link_configuration.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}


INCLUDEPATH += $$PWD/../../mavlink_cpp/V2/ardupilotmega

INCLUDEPATH += $$PWD/../
