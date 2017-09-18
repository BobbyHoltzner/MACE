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
    misc/data_2d.cpp \
    misc/data_3d.cpp \
    pose/orientation_2d.cpp \
    pose/orientation_3d.cpp \
    state_space/real_vector.cpp \
    geometry/polygon_2dc.cpp \
    pose/cartesian_position_3D.cpp \
    pose/cartesian_position_2D.cpp \
    state_space/real_vector_bounds.cpp \
    pose/geodetic_position_2D.cpp \
    geometry/cell_2DC.cpp \
    state_space/state_space.cpp \
    state_space/cartesian_2D_space.cpp \
    math/random_number_generator.cpp

HEADERS +=\
    base_global.h \
    math/helper_pi.h \
    pose/abstract_position.h \
    pose/cartesian_position_2D.h \
    pose/coordinate_frame.h \
    pose/base_position.h \
    pose/cartesian_position_3D.h \
    misc/data_2d.h \
    misc/data_3d.h \
    misc/data_forward_definition.h \
    geometry/base_polygon.h \
    state_space/real_vector.h \
    geometry/geometry_helper.h \
    pose/orientation_2D.h \
    pose/orientation_3D.h \
    math/math_forward.h \
    state_space/real_vector_bounds.h \
    misc/abstract_data.h \
    pose/geodetic_position_2D.h \
    geometry/polygon_2DC.h \
    geometry/cell_2DC.h \
    state_space/state_sampler.h \
    state_space/state_space_types.h \
    state_space/state_space.h \
    state_space/state.h \
    state_space/space_information.h \
    state_space/state_validity_checker.h \
    state_space/cartesian_2D_space.h \
    math/random_number_generator.h

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
    base_global.h
INSTALLS       += headers

#Header file copy
headers_geometry.path    = $$(MACE_ROOT)/include/base/geometry
headers_geometry.files   += \
    geometry/base_polygon.h \
    geometry/geometry_helper.h \
    geometry/polygon_2DC.h \
    geometry/cell_2DC.h
INSTALLS       += headers_geometry

headers_math.path    = $$(MACE_ROOT)/include/base/math
headers_math.files   += \
    math/helper_pi.h \
    math/math_forward.h
INSTALLS       += headers_math

headers_misc.path    = $$(MACE_ROOT)/include/base/misc
headers_misc.files   += \
    misc/abstract_data.h \
    misc/data_2d.h \
    misc/data_3d.h \
    misc/data_forward_definition.h
INSTALLS       += headers_misc

headers_pose.path    = $$(MACE_ROOT)/include/base/pose
headers_pose.files   += \
    pose/abstract_position.h \
    pose/cartesian_position_2D.h \
    pose/coordinate_frame.h \
    pose/base_position.h \
    pose/cartesian_position_3D.h \
    pose/orientation_2D.h \
    pose/orientation_3D.h \
    pose/geodetic_position_2D.h
INSTALLS       += headers_pose

headers_state_space.path    = $$(MACE_ROOT)/include/base/state_space
headers_state_space.files   += \
    state_space/real_vector.h \
    state_space/real_vector_bounds.h
INSTALLS       += headers_state_space

INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
