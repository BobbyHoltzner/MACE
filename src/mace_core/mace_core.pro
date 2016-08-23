#-------------------------------------------------
#
# Project created by QtCreator 2016-08-22T13:49:38
#
#-------------------------------------------------

QT       -= core gui

TARGET = mace_core
TEMPLATE = lib

DEFINES += MACE_CORE_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += mace_core.cpp \
    resource_number_generator.cpp \
    mace_data.cpp

HEADERS += mace_core.h\
        mace_core_global.h \
    metadata_vehicle.h \
    metadata_rta.h \
    resource_number_generator.h \
    module_base.h \
    i_module_command_RTA.h \
    i_module_command_vehicle.h \
    i_module_events_rta.h \
    i_module_events_vehicle.h \
    optional_parameter.h \
    vehicle_data.h \
    i_module_command_path_planning.h \
    i_module_events_path_planning.h \
    metadata_path_planning.h \
    mace_data.h \
    observation_history.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}
