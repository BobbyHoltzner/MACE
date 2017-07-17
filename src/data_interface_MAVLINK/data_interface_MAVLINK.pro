#-------------------------------------------------
#
# Project created by QtCreator 2017-07-06T10:01:40
#
#-------------------------------------------------
QT += serialport
QT += network
QT       -= core gui

TARGET = data_interface_MAVLINK
TEMPLATE = lib

DEFINES += DATA_INTERFACE_MAVLINK_LIBRARY

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
    vehicle_object_mavlink.cpp \
    callback_interface_data_mavlink.cpp \
    command_interface_mavlink.cpp \
    state_data_mavlink.cpp \
    mission_data_mavlink.cpp \
    parse_mavlink.cpp \
    mission_controller_mavlink.cpp \
    MACE_to_MAVLINK/helper_mission_mace_to_mavlink.cpp \
    MAVLINK_to_MACE/helper_mission_mavlink_to_mace.cpp \
    components/ardupilot_component_flight_mode.cpp \
    command_controller_mavlink.cpp

HEADERS +=\
        data_interface_mavlink_global.h \
    vehicle_object_mavlink.h \
    callback_interface_data_mavlink.h \
    command_interface_mavlink.h \
    state_data_mavlink.h \
    mission_data_mavlink.h \
    mission_controller_mavlink.h \
    MACE_to_MAVLINK/helper_mission_mace_to_mavlink.h \
    MAVLINK_to_MACE/helper_mission_mavlink_to_mace.h \
    generic/helper_previous_transmission.h \
    generic/comms_item.h \
    components/ardupilot_component_flight_mode.h \
    command_controller_mavlink.h \
    generic/command_item.h \
    generic/helper_previous_command_mavlink.h


# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data_interface_MAVLINK.lib release/data_interface_MAVLINK.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data_interface_MAVLINK.lib debug/data_interface_MAVLINK.dll
INSTALLS += lib


#Header file copy
headers.path    = $$(MACE_ROOT)/include/data_interface_MAVLINK
headers.files   += \
    callback_interface_data_mavlink.h \
    command_controller_mavlink.h \
    command_interface_mavlink.h \
    data_interface_mavlink_global.h \
    mission_controller_mavlink.h \
    mission_data_mavlink.h \
    state_data_mavlink.h \
    vehicle_object_mavlink.h
INSTALLS       += headers

#Header file copy
headers_generic.path    = $$(MACE_ROOT)/include/data_interface_MAVLINK/generic
headers_generic.files   += \
    generic/command_item.h \
    generic/comms_item.h \
    generic/helper_previous_transmission.h
INSTALLS       += headers_generic

#Header file copy
headers_components.path    = $$(MACE_ROOT)/include/data_interface_MAVLINK/components
headers_components.files   += \
    components/ardupilot_component_flight_mode.h
INSTALLS       += headers_components

#Header file copy
headers_MACE_to_MAVLINK.path    = $$(MACE_ROOT)/include/data_interface_MAVLINK/MACE_to_MAVLINK
headers_MACE_to_MAVLINK.files   += \
    MACE_to_MAVLINK/mission_mace_to_mavlink.h \
    MACE_to_MAVLINK/helper_mission_mace_to_mavlink.h
INSTALLS       += headers_MACE_to_MAVLINK

#Header file copy
headers_MAVLINK_to_MACE.path    = $$(MACE_ROOT)/include/data_interface_MAVLINK/MAVLINK_to_MACE
headers_MAVLINK_to_MACE.files   += \
    MAVLINK_to_MACE/mission_mavlink_to_mace.h \
    MAVLINK_to_MACE/helper_mission_mavlink_to_mace.h
INSTALLS       += headers_MAVLINK_to_MACE

INCLUDEPATH += $$PWD/../../speedLog/
INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/
INCLUDEPATH += $$PWD/../../mavlink_cpp/MAVLINK_BASE/ardupilotmega/
INCLUDEPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../comms/release/ -lcomms
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../comms/debug/ -lcomms
else:unix:!macx: LIBS += -L$$OUT_PWD/../comms/ -lcomms

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item/release/ -ldata_generic_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item/debug/ -ldata_generic_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_item/ -ldata_generic_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item_topic/release/ -ldata_generic_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item_topic/debug/ -ldata_generic_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_item_topic/ -ldata_generic_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item/release/ -ldata_generic_state_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item/debug/ -ldata_generic_state_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_state_item/ -ldata_generic_state_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/release/ -ldata_generic_state_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/debug/ -ldata_generic_state_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/ -ldata_generic_state_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item/release/ -ldata_generic_command_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item/debug/ -ldata_generic_command_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_command_item/ -ldata_generic_command_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item_topic/release/ -ldata_generic_command_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item_topic/debug/ -ldata_generic_command_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_command_item_topic/ -ldata_generic_command_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/release/ -ldata_generic_mission_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/debug/ -ldata_generic_mission_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/ -ldata_generic_mission_item_topic

INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3

