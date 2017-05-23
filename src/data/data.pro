#-------------------------------------------------
#
# Project created by QtCreator 2016-12-08T21:09:09
#
#-------------------------------------------------
QT -= core gui

TARGET = data
TEMPLATE = lib

DEFINES += DATA_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += \
    timer.cpp \
    system_description.cpp \
    environment_time.cpp \
    mission_key.cpp \
    mission_key_change.cpp

HEADERS += data_global.h \
    i_topic_component_data_object.h \
    topic_data_object_collection.h \
    coordinate_frame.h \
    vehicle_command_types.h \
    vehicle_types.h \
    timer.h \
    operating_mode.h \
    speed_frame.h \
    autopilot_types.h \
    controller_state.h \
    system_description.h \
    mission_type.h \
    environment_time.h \
    date.h \
    mission_key.h \
    mission_key_change.h \
    data_get_set_notifier.h \
    mission_ack.h \
    threadmanager.h \
    system_type.h \
    comms_protocol.h \
    gps_fix_type.h \
    status_severity_type.h \
    command_item_type.h \
    command_ack_type.h \
    mission_command.h
    loiter_direction.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data.lib release/data.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data.lib debug/data.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/data
headers.files   += $$HEADERS
INSTALLS       += headers

INCLUDEPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon
