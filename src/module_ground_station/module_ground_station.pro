#-------------------------------------------------
#
# Project created by QtCreator 2016-11-10T10:01:30
#
#-------------------------------------------------
QT       += network
QT += widgets
QT       -= gui

TARGET = module_ground_station
TEMPLATE = lib

DEFINES += MODULE_GROUND_STATION_LIBRARY

SOURCES += module_ground_station.cpp

HEADERS += module_ground_station.h\
        module_ground_station_global.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/module_ground_station.lib release/module_ground_station.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/module_ground_station.lib debug/module_ground_station.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/module_ground_station
headers.files   += $$HEADERS
INSTALLS       += headers

INCLUDEPATH += $$PWD/../../mavlink_cpp/V2/common
INCLUDEPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../comms/release/ -lcomms
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../comms/debug/ -lcomms
else:unix: LIBS += -L$$OUT_PWD/../comms/ -lcomms

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_topics/release/ -ldata_generic_state_topics
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_topics/debug/ -ldata_generic_state_topics
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_state_topics/ -ldata_generic_state_topics

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_MAVLINK/release/ -ldata_vehicle_MAVLINK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_MAVLINK/debug/ -ldata_vehicle_MAVLINK
else:unix: LIBS += -L$$OUT_PWD/../data_vehicle_MAVLINK/ -ldata_vehicle_MAVLINK

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_ardupilot/release/ -ldata_vehicle_ardupilot
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_ardupilot/debug/ -ldata_vehicle_ardupilot
else:unix: LIBS += -L$$OUT_PWD/../data_vehicle_ardupilot/ -ldata_vehicle_ardupilot


unix{
    EigenInclude = $$system(pkg-config --cflags eigen3)
    EigenInclude = $$replace(EigenInclude, "-I", "")/eigen3
    INCLUDEPATH += $$EigenInclude
}
win32{
    INCLUDEPATH += "C:\Program Files (x86)\Eigen\include\eigen3"
}


