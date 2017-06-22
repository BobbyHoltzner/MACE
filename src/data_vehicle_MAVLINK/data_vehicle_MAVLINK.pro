#-------------------------------------------------
#
# Project created by QtCreator 2017-01-10T14:37:24
#
#-------------------------------------------------

QT       -= core gui

TARGET = data_vehicle_MAVLINK
TEMPLATE = lib

DEFINES += DATA_VEHICLE_MAVLINK_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += \
    mavlink_parser.cpp \
    data_container_mavlink.cpp \
    vehicle_object_mavlink.cpp \
    MACE_to_MAVLINK/mission_mace_to_mavlink.cpp \
    MACE_to_MAVLINK/state_mace_to_mavlink.cpp \
    MACE_to_MAVLINK/generic_mace_to_mavlink.cpp \
    MACE_to_MAVLINK/command_mace_to_mavlink.cpp \
    MAVLINK_to_MACE/mission_mavlink_to_mace.cpp \
    MAVLINK_to_MACE/command_mavlink_to_mace.cpp \
    MAVLINK_to_MACE/generic_mavlink_to_mace.cpp \
    MAVLINK_to_MACE/state_mavlink_to_mace.cpp \
    MAVLINK_to_MACE/container_mavlink_to_mace.cpp \
    MACE_to_MAVLINK/container_mace_to_mavlink.cpp

HEADERS +=\
    mavlink_parser.h \
    vehicle_object_mavlink.h \
    data_container_mavlink.h \
    data_vehicle_mavlink_global.h \
    MACE_to_MAVLINK/mission_mace_to_mavlink.h \
    MACE_to_MAVLINK/state_mace_to_mavlink.h \
    MACE_to_MAVLINK/generic_mace_to_mavlink.h \
    MACE_to_MAVLINK/command_mace_to_mavlink.h \
    MACE_to_MAVLINK/container_mace_to_mavlink.h \
    MAVLINK_to_MACE/mission_mavlink_to_mace.h \
    MAVLINK_to_MACE/command_mavlink_to_mace.h \
    MAVLINK_to_MACE/generic_mavlink_to_mace.h \
    MAVLINK_to_MACE/state_mavlink_to_mace.h \
    MAVLINK_to_MACE/container_mavlink_to_mace.h


# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data_vehicle_MAVLINK.lib release/data_vehicle_MAVLINK.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data_vehicle_MAVLINK.lib debug/data_vehicle_MAVLINK.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/data_vehicle_MAVLINK
headers.files   += \
    mavlink_parser.h \
    components.h \
    vehicle_object_mavlink.h \
    data_container_mavlink.h \
    data_vehicle_mavlink_global.h
INSTALLS       += headers

#Header file copy
headers_MACE_to_MAVLINK.path    = $$(MACE_ROOT)/include/data_vehicle_MAVLINK/MACE_to_MAVLINK
headers_MACE_to_MAVLINK.files   += \
    MACE_to_MAVLINK/command_mace_to_mavlink.h \
    MACE_to_MAVLINK/container_mace_to_mavlink.h \
    MACE_to_MAVLINK/generic_mace_to_mavlink.h \
    MACE_to_MAVLINK/mission_mace_to_mavlink.h \
    MACE_to_MAVLINK/state_mace_to_mavlink.h
INSTALLS       += headers_MACE_to_MAVLINK

#Header file copy
headers_MAVLINK_to_MACE.path    = $$(MACE_ROOT)/include/data_vehicle_MAVLINK/MAVLINK_to_MACE
headers_MAVLINK_to_MACE.files   += \
    MAVLINK_to_MACE/command_mavlink_to_mace.h \
    MAVLINK_to_MACE/container_mavlink_to_mace.h \
    MAVLINK_to_MACE/generic_mavlink_to_mace.h \
    MAVLINK_to_MACE/mission_mavlink_to_mace.h \
    MAVLINK_to_MACE/state_mavlink_to_mace.h
INSTALLS       += headers_MAVLINK_to_MACE

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

