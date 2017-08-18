#-------------------------------------------------
#
# Project created by QtCreator 2017-08-17T19:32:56
#
#-------------------------------------------------

QT       -= core gui

TARGET = maps
TEMPLATE = lib

DEFINES += MAPS_LIBRARY

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

SOURCES += \
    dynamic_2D_grid.cpp

HEADERS +=\
        maps_global.h \
    dynamic_2D_grid.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/maps.lib release/maps.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/maps.lib debug/maps.dll
INSTALLS += lib
