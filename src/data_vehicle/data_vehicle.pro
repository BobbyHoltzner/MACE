#-------------------------------------------------
#
# Project created by QtCreator 2016-10-30T18:26:26
#
#-------------------------------------------------

QT       -= core gui

TARGET = data_vehicle
TEMPLATE = lib

DEFINES += DATA_VEHICLE_LIBRARY

SOURCES += vehicle_state_data.cpp \
        vehicle_data.cpp \
        Arducopter/arducopter_data.cpp \
    Arducopter/arducopter_properties.cpp \
    Arducopter/arducopter_gps.cpp \
    Arducopter/arducopter_attitude.cpp \
    Arducopter/arducopter_main.cpp


HEADERS += vehicle_state_data.h\
        data_vehicle_global.h \
        vehicle_data.h \
        Arducopter/arducopter_data.h \
        Arducopter/arducopter_collection.h \
    Arducopter/arducopter_properties.h \
    Arducopter/arducopter_gps.h \
    Arducopter/arducopter_attitude.h \
    Arducopter/arducopter_main.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data_vehicle.lib release/data_vehicle.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data_vehicle.lib debug/data_vehicle.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/data_vehicle
headers.files   += $$HEADERS
INSTALLS       += headers

INCLUDEPATH += $$(MACE_ROOT)/include

INCLUDEPATH += $$PWD/../../mavlink_cpp/V2/ardupilotmega
