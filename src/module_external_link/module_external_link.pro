#-------------------------------------------------
#
# Project created by QtCreator 2017-01-13T16:56:01
#
#-------------------------------------------------

QT       -= core gui

TARGET = module_external_link
TEMPLATE = lib

DEFINES += MODULE_EXTERNAL_LINK_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += module_external_link.cpp

HEADERS += module_external_link.h\
        module_external_link_global.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/module_vehicle_MAVLINK.lib release/module_vehicle_MAVLINK.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/module_vehicle_MAVLINK.lib debug/module_vehicle_MAVLINK.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/module_vehicle_MAVLINK
headers.files   += \
        module_vehicle_mavlink.h \
        module_vehicle_mavlink_global.h \
        generic_message_definition_mavlink.h
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

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_generic/release/ -ldata_vehicle_generic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_generic/debug/ -ldata_vehicle_generic
else:unix: LIBS += -L$$OUT_PWD/../data_vehicle_generic/ -ldata_vehicle_generic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_MAVLINK/release/ -ldata_vehicle_MAVLINK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_MAVLINK/debug/ -ldata_vehicle_MAVLINK
else:unix: LIBS += -L$$OUT_PWD/../data_vehicle_MAVLINK/ -ldata_vehicle_MAVLINK

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_ardupilot/release/ -ldata_vehicle_ardupilot
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_ardupilot/debug/ -ldata_vehicle_ardupilot
else:unix: LIBS += -L$$OUT_PWD/../data_vehicle_ardupilot/ -ldata_vehicle_ardupilot

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_commands/release/ -ldata_vehicle_commands
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_commands/debug/ -ldata_vehicle_commands
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_vehicle_commands/ -ldata_vehicle_commands

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_sensors/release/ -ldata_vehicle_sensors
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_sensors/debug/ -ldata_vehicle_sensors
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_vehicle_sensors/ -ldata_vehicle_sensors

unix{
    EigenInclude = $$system(pkg-config --cflags eigen3)
    EigenInclude = $$replace(EigenInclude, "-I", "")/eigen3
    INCLUDEPATH += $$EigenInclude
}
win32{
    INCLUDEPATH += "C:\Program Files (x86)\Eigen\include\eigen3"
}

