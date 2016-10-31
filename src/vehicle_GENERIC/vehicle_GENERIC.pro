#-------------------------------------------------
#
# Project created by QtCreator 2016-10-28T11:14:24
#
#-------------------------------------------------

QT       -= core gui

TARGET = vehicle_GENERIC
TEMPLATE = lib

DEFINES += VEHICLE_GENERIC_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += vehicle_generic.cpp

HEADERS += vehicle_generic.h\
        vehicle_generic_global.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/vehicle_GENERIC.lib release/vehicle_GENERIC.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/vehicle_GENERIC.lib debug/vehicle_GENERIC.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/vehicle_GENERIC
headers.files   += $$HEADERS
INSTALLS       += headers

INCLUDEPATH += $$(MACE_ROOT)/include


INCLUDEPATH += $$PWD/../../mavlink_cpp/V2/ardupilotmega


unix{
    EigenInclude = $$system(pkg-config --cflags eigen3)
    EigenInclude = $$replace(EigenInclude, "-I", "")/eigen3
    INCLUDEPATH += $$EigenInclude
}
win32{
    INCLUDEPATH += "C:\Program Files (x86)\Eigen\include\eigen3"
}

