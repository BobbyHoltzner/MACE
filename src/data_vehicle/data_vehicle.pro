#-------------------------------------------------
#
# Project created by QtCreator 2016-11-09T09:11:59
#
#-------------------------------------------------

QT       -= core gui

TARGET = data_vehicle
TEMPLATE = lib

DEFINES += DATA_VEHICLE_LIBRARY

SOURCES += data_vehicle.cpp

HEADERS += data_vehicle.h\
        data_vehicle_global.h

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


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

INCLUDEPATH += $$PWD/../../mavlink_cpp/V2/ardupilotmega


unix{
    EigenInclude = $$system(pkg-config --cflags eigen3)
    EigenInclude = $$replace(EigenInclude, "-I", "")/eigen3
    INCLUDEPATH += $$EigenInclude
}
win32{
    INCLUDEPATH += "C:\Program Files (x86)\Eigen\include\eigen3"
}


INCLUDEPATH += $$PWD/../
DEPENDPATH += $$PWD/../
