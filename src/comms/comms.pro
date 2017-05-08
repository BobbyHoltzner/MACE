#-------------------------------------------------
#
# Project created by QtCreator 2016-08-31T13:55:42
#
#-------------------------------------------------
QT += serialport
QT += network
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
    protocol_mavlink.cpp \
    udp_configuration.cpp \
    udp_link.cpp

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
    comms_events.h \
    udp_configuration.h \
    udp_link.h

INCLUDEPATH += $$PWD/../../mavlink_cpp/Stable/ardupilotmega/
INCLUDEPATH += $$PWD/../

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/comms.lib release/comms.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/comms.lib debug/comms.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/comms
headers.files   += $$HEADERS
INSTALLS       += headers

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

