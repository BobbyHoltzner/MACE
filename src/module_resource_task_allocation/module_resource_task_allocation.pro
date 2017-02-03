#-------------------------------------------------
#
# Project created by QtCreator 2017-01-31T19:28:14
#
#-------------------------------------------------

QT       -= core gui

TARGET = module_resource_task_allocation
TEMPLATE = lib

QMAKE_CXXFLAGS += -std=c++11

DEFINES += MODULE_RESOURCE_TASK_ALLOCATION_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += module_rta.cpp

HEADERS += module_rta.h\
        module_resource_task_allocation_global.h
# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/module_resource_task_allocation.lib release/module_resource_task_allocation.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/module_resource_task_allocation.lib debug/module_resource_task_allocation.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/module_resource_task_allocation
headers.files   += \
        module_resource_task_allocation_global.h\
        module_rta.h
INSTALLS       += headers


INCLUDEPATH += $$PWD/../../mavlink_cpp/V2/common
INCLUDEPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

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
