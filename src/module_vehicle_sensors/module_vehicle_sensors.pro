#-------------------------------------------------
#
# Project created by QtCreator 2017-01-24T19:59:11
#
#-------------------------------------------------

QT       -= core gui

TARGET = module_vehicle_sensors
TEMPLATE = lib

DEFINES += MODULE_VEHICLE_SENSORS_LIBRARY

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

SOURCES += module_vehicle_sensors.cpp

HEADERS += module_vehicle_sensors.h\
        module_vehicle_sensors_global.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/module_vehicle_sensors.lib release/module_vehicle_sensors.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/module_vehicle_sensors.lib debug/module_vehicle_sensors.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/module_vehicle_sensors
headers.files   += \
        module_vehicle_sensors.h \
        module_vehicle_sensors_global.h
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

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/release/ -ldata_generic_state_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/debug/ -ldata_generic_state_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/ -ldata_generic_state_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item/release/ -ldata_generic_state_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item/debug/ -ldata_generic_state_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_state_item/ -ldata_generic_state_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item/release/ -ldata_generic_mission_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item/debug/ -ldata_generic_mission_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_mission_item/ -ldata_generic_mission_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/release/ -ldata_generic_mission_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/debug/ -ldata_generic_mission_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/ -ldata_generic_mission_item_topic

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


