#-------------------------------------------------
#
# Project created by QtCreator 2016-12-08T21:09:09
#
#-------------------------------------------------
QT -= gui

TARGET = data
TEMPLATE = lib

DEFINES += DATA_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

CONFIG += object_parallel_to_source

SOURCES += \
    system_description.cpp \
    environment_time.cpp \
    topic_components/altitude.cpp \
    topic_prototypes/altitude.cpp \
    topic_prototypes/position_georeference.cpp \
    topic_components/position_global.cpp \
    topic_components/position_local.cpp \
    topic_prototypes/position_cartesian_3d.cpp \
    topic_components/topic_component_string.cpp \
    topic_components/topic_component_void.cpp

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
    data_get_set_notifier.h \
    system_type.h \
    comms_protocol.h \
    command_ack_type.h \
    mission_command.h \
    mission_execution_state.h \
    command_validity_type.h \
    controller_comms_state.h \
    environment_time.h \
    topic_components/altitude.h \
    reference_altitude.h \
    topic_prototypes/altitude.h \
    topic_prototypes/position_georeference.h \
    reference_georeference.h \
    topic_components/position_global.h \
    reference_cartesian.h \
    topic_components/position_local.h \
    topic_prototypes/position_cartesian_3d.h \
    topic_components/topic_component_string.h \
    topic_components/topic_component_void.h
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
INSTALL_PREFIX = $$(MACE_ROOT)/include/data
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)

INCLUDEPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon
