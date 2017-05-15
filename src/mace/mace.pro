TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
QT += serialport
QT += network

TARGET = MACE
QMAKE_CXXFLAGS += -std=c++11


SOURCES += main.cpp \
    data_interpolation.cpp \
    configuration_reader_xml.cpp \
    pugixml.cpp


HEADERS += \
    data_interpolation.h \
    configuration_reader_xml.h \
    pugixml.hpp \
    pugiconfig.hpp \
    module_collection.h


# Copy Files
target.path = $$(MACE_ROOT)/bin
INSTALLS += target

INCLUDEPATH += $$(MACE_ROOT)/include


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../comms/release/ -lcomms
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../comms/debug/ -lcomms
else:unix: LIBS += -L$$OUT_PWD/../comms/ -lcomms

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../commsMAVLINK/release/ -lcommsMAVLINK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../commsMAVLINK/debug/ -lcommsMAVLINK
else:unix: LIBS += -L$$OUT_PWD/../commsMAVLINK/ -lcommsMAVLINK



win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../commsMACE/release/ -lcommsMACE
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../commsMACE/debug/ -lcommsMACE
else:unix: LIBS += -L$$OUT_PWD/../commsMACE/ -lcommsMACE

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../commsMACEHelper/release/ -lcommsMACEHelper
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../commsMACEHelper/debug/ -lcommsMACEHelper
else:unix:!macx: LIBS += -L$$OUT_PWD/../commsMACEHelper/ -lcommsMACEHelper



win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_comms/release/ -ldata_comms
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_comms/debug/ -ldata_comms
else:unix: LIBS += -L$$OUT_PWD/../data_comms/ -ldata_comms

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




win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_path_planning_NASAPhase2/release/ -lmodule_path_planning_NASAPhase2
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_path_planning_NASAPhase2/debug/ -lmodule_path_planning_NASAPhase2
else:unix: LIBS += -L$$OUT_PWD/../module_path_planning_NASAPhase2/ -lmodule_path_planning_NASAPhase2

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_resource_task_allocation/release/ -lmodule_resource_task_allocation
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_resource_task_allocation/debug/ -lmodule_resource_task_allocation
else:unix:!macx: LIBS += -L$$OUT_PWD/../module_resource_task_allocation/ -lmodule_resource_task_allocation

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_ground_station/release/ -lmodule_ground_station
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_ground_station/debug/ -lmodule_ground_station
else:unix:!macx: LIBS += -L$$OUT_PWD/../module_ground_station/ -lmodule_ground_station

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_external_link/release/ -lmodule_external_link
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_external_link/debug/ -lmodule_external_link
else:unix:!macx: LIBS += -L$$OUT_PWD/../module_external_link/ -lmodule_external_link


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_sensors/release/ -ldata_vehicle_sensors
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_sensors/debug/ -ldata_vehicle_sensors
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_vehicle_sensors/ -ldata_vehicle_sensors

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_sensors/release/ -lmodule_vehicle_sensors
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_sensors/debug/ -lmodule_vehicle_sensors
else:unix:!macx: LIBS += -L$$OUT_PWD/../module_vehicle_sensors/ -lmodule_vehicle_sensors


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_generic/release/ -lmodule_vehicle_generic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_generic/debug/ -lmodule_vehicle_generic
else:unix: LIBS += -L$$OUT_PWD/../module_vehicle_generic/ -lmodule_vehicle_generic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/release/ -lmodule_vehicle_MAVLINK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/debug/ -lmodule_vehicle_MAVLINK
else:unix: LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/ -lmodule_vehicle_MAVLINK

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_MAVLINK/release/ -ldata_vehicle_MAVLINK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_MAVLINK/debug/ -ldata_vehicle_MAVLINK
else:unix: LIBS += -L$$OUT_PWD/../data_vehicle_MAVLINK/ -ldata_vehicle_MAVLINK

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_ardupilot/release/ -lmodule_vehicle_ardupilot
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_ardupilot/debug/ -lmodule_vehicle_ardupilot
else:unix: LIBS += -L$$OUT_PWD/../module_vehicle_ardupilot/ -lmodule_vehicle_ardupilot

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_ardupilot/release/ -ldata_vehicle_ardupilot
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_ardupilot/debug/ -ldata_vehicle_ardupilot
else:unix: LIBS += -L$$OUT_PWD/../data_vehicle_ardupilot/ -ldata_vehicle_ardupilot

INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/
INCLUDEPATH += $$PWD/../../mavlink_cpp/MAVLINK_BASE/ardupilotmega/
INCLUDEPATH += $$PWD/../

unix{
    EigenInclude = $$system(pkg-config --cflags eigen3)
    EigenInclude = $$replace(EigenInclude, "-I", "")/eigen3
    INCLUDEPATH += $$EigenInclude
}
win32{
    INCLUDEPATH += "C:\Program Files (x86)\Eigen\include\eigen3"
}
