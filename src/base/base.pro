#-------------------------------------------------
#
# Project created by QtCreator 2017-08-27T11:33:13
#
#-------------------------------------------------

QT       -= core gui

TARGET = base
TEMPLATE = lib

DEFINES += BASE_LIBRARY

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
    geometry/polygon2d.cpp \
    pose/point_2d.cpp \
    pose/point_3d.cpp

HEADERS +=\
        base_global.h \
    pose/abstract_point.h \
    pose/axis_point_helper.h \
    pose/point.h \
    pose/point_2d.h \
    pose/point_3d.h \
    pose/point_forward_definition.h \
    abstractpostion.h \
    abstractposition2.h \
    math/array_numeric.h \
    math/helper_pi.h \
    geometry/polygon2d.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/base.lib release/base.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/base.lib debug/base.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/base
headers.files   += \
        base_global.h \
    abstractpostion.h
INSTALLS       += headers

#Header file copy
headers_pose.path    = $$(MACE_ROOT)/include/base/pose
headers_pose.files   += \
    pose/abstract_point.h \
    pose/axis_point_helper.h \
    pose/point.h \
    pose/point_2d.h \
    pose/point_3d.h \
    pose/point_forward_definition.h
INSTALLS       += headers_pose


INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
