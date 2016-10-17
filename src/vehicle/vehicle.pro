#-------------------------------------------------
#
# Project created by QtCreator 2016-10-12T09:55:40
#
#-------------------------------------------------

QT       -= core gui

TARGET = vehicle
TEMPLATE = lib

DEFINES += VEHICLE_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += vehicle.cpp \
    position.cpp \
    orientation.cpp \
    gpsinfo.cpp \
    fuelinfo.cpp \
    radioinfo.cpp \
    properties.cpp

HEADERS += vehicle.h\
        vehicle_global.h \
    position.h \
    orientation.h \
    gpsinfo.h \
    fuelinfo.h \
    radioinfo.h \
    properties.h

unix {
    target.path = /usr/local/lib
    INSTALLS += target
}


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

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
