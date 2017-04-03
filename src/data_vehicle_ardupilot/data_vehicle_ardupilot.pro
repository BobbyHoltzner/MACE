#-------------------------------------------------
#
# Project created by QtCreator 2017-01-12T13:32:38
#
#-------------------------------------------------

QT       -= core gui

TARGET = data_vehicle_ardupilot
TEMPLATE = lib

DEFINES += DATA_VEHICLE_ARDUPILOT_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += \
    components/vehicle_flightMode.cpp \
    MACE_to_ARDUPILOT/command_mace_to_ardupilot.cpp \
    MACE_to_ARDUPILOT/generic_mace_to_ardupilot.cpp \
    MACE_to_ARDUPILOT/mission_mace_to_ardupilot.cpp \
    MACE_to_ARDUPILOT/state_mace_to_ardupilot.cpp \
    ARDUPILOT_to_MACE/command_ardupilot_to_mace.cpp \
    ARDUPILOT_to_MACE/generic_ardupilot_to_mace.cpp \
    ARDUPILOT_to_MACE/mission_ardupilot_to_mace.cpp \
    ARDUPILOT_to_MACE/state_ardupilot_to_mace.cpp \
    data_container_ardupilot.cpp \
    ardupilot_parser.cpp \
    vehicle_object_ardupilot.cpp \
    MACE_to_ARDUPILOT/container_mace_to_ardupilot.cpp \
    ARDUPILOT_to_MACE/container_ardupilot_to_mace.cpp

HEADERS +=\
        data_vehicle_ardupilot_global.h \
    components.h \
    components/vehicle_flightMode.h \
    MACE_to_ARDUPILOT/command_mace_to_ardupilot.h \
    MACE_to_ARDUPILOT/container_mace_to_ardupilot.h \
    MACE_to_ARDUPILOT/generic_mace_to_ardupilot.h \
    MACE_to_ARDUPILOT/mission_mace_to_ardupilot.h \
    MACE_to_ARDUPILOT/state_mace_to_ardupilot.h \
    ARDUPILOT_to_MACE/state_ardupilot_to_mace.h \
    ARDUPILOT_to_MACE/mission_ardupilot_to_mace.h \
    ARDUPILOT_to_MACE/generic_ardupilot_to_mace.h \
    ARDUPILOT_to_MACE/command_ardupilot_to_mace.h \
    ARDUPILOT_to_MACE/container_ardupilot_to_mace.h \
    data_container_ardupilot.h \
    ardupilot_parser.h \
    vehicle_object_ardupilot.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data_vehicle_ardupilot.lib release/data_vehicle_ardupilot.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data_vehicle_ardupilot.lib debug/data_vehicle_ardupilot.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/data_vehicle_ardupilot
headers.files   += \
        data_vehicle_ardupilot_global.h \
    components.h \
    components/vehicle_flightMode.h \
    data_container_ardupilot.h \
    ardupilot_parser.h \
    vehicle_object_ardupilot.h \

INSTALLS       += headers

#Header file copy
headers_Components.path    = $$(MACE_ROOT)/include/data_vehicle_ardupilot/Components
headers_Components.files   += \
        components/vehicle_flightMode.h \
        components/vehicle_operating_status.h
INSTALLS       += headers_Components

#Header file copy
headers_MACE_to_ARDUPILOT.path    = $$(MACE_ROOT)/include/data_vehicle_ardupilot/MACE_to_ARDUPILOT
headers_MACE_to_ARDUPILOT.files   += \
    MACE_to_ARDUPILOT/command_mace_to_mavlink.h \
    MACE_to_ARDUPILOT/container_mace_to_ardupilot.h \
    MACE_to_ARDUPILOT/generic_mace_to_mavlink.h \
    MACE_to_ARDUPILOT/mission_mace_to_mavlink.h \
    MACE_to_ARDUPILOT/state_mace_to_mavlink.h
INSTALLS       += headers_MACE_to_ARDUPILOT

#Header file copy
headers_ARDUPILOT_to_MACE.path    = $$(MACE_ROOT)/include/data_vehicle_ardupilot/ARDUPILOT_to_MACE
headers_ARDUPILOT_to_MACE.files   += \
    ARDUPILOT_to_MACE/command_mavlink_to_mace.h \
    ARDUPILOT_to_MACE/container_ardupilot_to_mace.h \
    ARDUPILOT_to_MACE/generic_mavlink_to_mace.h \
    ARDUPILOT_to_MACE/mission_mavlink_to_mace.h \
    ARDUPILOT_to_MACE/state_mavlink_to_mace.h
INSTALLS       += headers_ARDUPILOT_to_MACE



INCLUDEPATH += $$PWD/../../mavlink_cpp/V2/ardupilotmega
INCLUDEPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

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

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item/release/ -ldata_generic_mission_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item/debug/ -ldata_generic_mission_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_mission_item/ -ldata_generic_mission_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/release/ -ldata_generic_mission_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/debug/ -ldata_generic_mission_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/ -ldata_generic_mission_item_topic


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_MAVLINK/release/ -ldata_vehicle_MAVLINK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_MAVLINK/debug/ -ldata_vehicle_MAVLINK
else:unix: LIBS += -L$$OUT_PWD/../data_vehicle_MAVLINK/ -ldata_vehicle_MAVLINK


unix{
    EigenInclude = $$system(pkg-config --cflags eigen3)
    EigenInclude = $$replace(EigenInclude, "-I", "")/eigen3
    INCLUDEPATH += $$EigenInclude
}
win32{
    INCLUDEPATH += "C:\Program Files (x86)\Eigen\include\eigen3"
}
