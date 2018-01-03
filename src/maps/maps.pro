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
    bounded_2D_grid.cpp \
    base_grid_map.cpp \
    dynamic_2D_grid.cpp \
    iterators/circle_iterator.cpp \
    iterators/grid_map_iterator.cpp

HEADERS +=\
        maps_global.h \
    dynamic_2D_grid.h \
    bounded_2D_grid.h \
    base_grid_map.h \
    iterators/circle_iterator.h \
    iterators/grid_map_iterator.h \
    data_2d_grid.h

#Header file copy
headers_maps.path    = $$(MACE_ROOT)/include/maps
headers_maps.files   += \
        base_grid_map.h \
        bounded_2D_grid.h \
        data_2d_grid.h \
        dynamic_2D_grid.h \
        maps_global.h
INSTALLS       += headers_maps

#Header file copy
headers_iterators.path    = $$(MACE_ROOT)/include/iterators
headers_iterators.files   += \
        iterators/circle_iterator.h \
        iterators/grid_map_iterator.h
INSTALLS       += headers_iterators

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/maps.lib release/maps.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/maps.lib debug/maps.dll
INSTALLS += lib

#Necessary includes
INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase
