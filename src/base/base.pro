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
    pose/geodetic_posiiton.cpp \
    misc/data_2d.cpp \
    misc/data_3d.cpp \
    geometry/base_polygon.cpp \
    pose/cartesian_position_temp.cpp

HEADERS +=\
    base_global.h \
    math/array_numeric.h \
    math/helper_pi.h \
    pose/abstract_position.h \
    pose/geodetic_position.h \
    pose/cartesian_position_2D.h \
    pose/coordinate_frame.h \
    pose/base_position.h \
    pose/cartesian_position_3D.h \
    misc/abstract_data.h \
    misc/data_2d.h \
    misc/data_3d.h \
    misc/data_forward_definition.h \
    geometry/base_polygon.h \
    pose/cartesian_position_temp.h
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
    pose/abstract_position.h \
    pose/geodetic_position.h \
    pose/cartesian_position_2D.h \
    pose/coordinate_frame.h \
    pose/base_position.h \
    pose/cartesian_position_3D.h \
    pose/cartesian_position_temp.h

INSTALLS       += headers_pose

headers_misc.path    = $$(MACE_ROOT)/include/base/misc
headers_misc.files   += \
    misc/abstract_data.h \
    misc/data_2d.h \
    misc/data_3d.h \
    misc/data_forward_definition.h
INSTALLS       += headers_pose


INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
