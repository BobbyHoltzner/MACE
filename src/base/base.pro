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
    pose/cartesian_position_3D.cpp \
    pose/cartesian_position_2D.cpp \
    state_space/real_vector_bounds.cpp \
    pose/geodetic_position_2D.cpp \
    state_space/state_space.cpp \
    state_space/cartesian_2D_space.cpp \
    math/random_number_generator.cpp \
    math/cost.cpp \
    state_space/goal_state.cpp \
    state_space/space_information.cpp \
    state_space/motion_validity_check.cpp \
    state_space/discrete_motion_validity_check.cpp \
    state_space/special_validity_check.cpp \
    geometry/cell_2DC.cpp \
    geometry/polygon_2dc.cpp \
    misc/data_1d.cpp \
    state_space/start_state.cpp \
    pose/geodetic_position_3D.cpp \
    pose/dynamics_aid.cpp

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
    state_space/state_sampler.h \
    state_space/state_space_types.h \
    state_space/state_space.h \
    state_space/state.h \
    state_space/space_information.h \
    state_space/cartesian_2D_space.h \
    math/random_number_generator.h \
    math/cost.h \
    state_space/generic_goal.h \
    state_space/goal_state.h \
    state_space/abstract_motion_validity_check.h \
    state_space/abstract_state_validity_check.h \
    state_space/discrete_motion_validity_check.h \
    state_space/special_validity_check.h \
    geometry/base_line.h \
    geometry/base_line.h \
    geometry/base_polygon.h \
    geometry/cell_2DC.h \
    geometry/geometry_helper.h \
    geometry/polygon_2DC.h \
    geometry/base_line.h \
    pose/abstract_velocity.h \
    misc/data_1d.h \
    state_space/start_state.h \
    state_space/generic_start.h \
    pose/geodetic_position_3D.h \
    pose/dynamics_aid.h

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
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)


INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

