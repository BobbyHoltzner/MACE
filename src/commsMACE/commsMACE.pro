#-------------------------------------------------
#
# Project created by QtCreator 2017-05-01T13:55:43
#
#-------------------------------------------------
QT += serialport
QT += network
QT       -= core gui

TARGET = commsMACE
TEMPLATE = lib

DEFINES += COMMSMACE_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    comms_marshaler_mace.cpp \
    mavlink_configuration_mace.cpp \
    protocol_mavlink_mace.cpp \
    serial_configuration_mace.cpp \
    serial_link_mace.cpp \
    udp_configuration_mace.cpp \
    udp_link_mace.cpp \
    digimesh_configuration.cpp \
    digimesh_link.cpp

HEADERS +=\
        commsmace_global.h \
    comms_events_mace.h \
    comms_marshaler_mace.h \
    i_link_mace.h \
    i_link_events_mace.h \
    i_protocol_mace.h \
    i_protocol_events_mace.h \
    i_protocol_mavlink_events_mace.h \
    link_configuration_mace.h \
    mavlink_configuration_mace.h \
    protocol_configuration_mace.h \
    protocol_mavlink_mace.h \
    serial_configuration_mace.h \
    serial_link_mace.h \
    udp_configuration_mace.h \
    udp_link_mace.h \
    digimesh_configuration.h \
    digimesh_link.h

INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/
INCLUDEPATH += $$PWD/../

INCLUDEPATH += $$(MACE_DIGIMESH_WRAPPER)/include/
LIBS += -L$$(MACE_DIGIMESH_WRAPPER)/lib/ -lMACEDigiMeshWrapper

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/commsMACE.lib release/commsMACE.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/commsMACE.lib debug/commsMACE.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/commsMACE
headers.files   += $$HEADERS
INSTALLS       += headers


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata
