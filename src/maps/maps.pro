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
    iterators/grid_map_iterator.cpp \
    iterators/polygon_map_iterator.cpp \
    iterators/circle_map_iterator.cpp \
    iterators/generic_map_iterator.cpp \
    octomap_wrapper.cpp \
    occupancy_2d_grid_topic.cpp \
    occupancy_map_2D_inflated.cpp

HEADERS +=\
        maps_global.h \
    dynamic_2D_grid.h \
    bounded_2D_grid.h \
    base_grid_map.h \
    data_2d_grid.h \
    iterators/polygon_map_iterator.h \
    iterators/circle_map_iterator.h \
    iterators/grid_map_iterator.h \
    iterators/generic_map_iterator.h \
    dynamic_2D_grid.tpp \
    octomap_wrapper.h \
    octomap_sensor_definition.h \
    occupancy_2d_grid_topic.h \
    map_topic_components.h \
    octomap_2d_projection_definition.h \
    occupancy_map_2D_inflated.h \
    occupancy_definition.h

#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)


# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

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

#Necessary includes
INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../../tools/octomap/bin/ -loctomap -loctomath
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../../tools/octomap/bin/ -loctomap -loctomath
else:unix:!macx: LIBS += -L$$OUT_PWD/../../tools/octomap/lib/ -loctomap -loctomath

INCLUDEPATH += $$OUT_PWD/../../tools/octomap/octomap/include
