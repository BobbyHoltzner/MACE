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
    serial_configuration.cpp \
    mavlink_configuration.cpp \
    comms_marshaler.cpp \
    protocol_mavlink.cpp

HEADERS +=\
    i_link.h \
    serial_link.h \
    i_protocol.h \
    comms_global.h \
    i_link_events.h \
    serial_configuration.h \
    link_configuration.h \
    protocol_configuration.h \
    mavlink_configuration.h \
    comms_marshaler.h \
    i_protocol_events.h \
    i_protocol_mavlink_events.h \
    protocol_mavlink.h \
    comms_events.h

unix {
    target.path = /usr/local/lib
    INSTALLS += target
}


INCLUDEPATH += $$PWD/../../mavlink_cpp/V2/ardupilotmega

INCLUDEPATH += $$PWD/../
