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
    mace_data.cpp \
    module_command_initialization.cpp

HEADERS += mace_core.h\
        mace_core_global.h \
    metadata_vehicle.h \
    metadata_rta.h \
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
    observation_history.h \
    module_parameters.h \
    abstract_module_base.h \
    abstract_module_event_listeners.h \
    module_factory.h \
    abstract_module_base_vehicle_listener.h \
    matrix_operations.h \
    command_marshler.h

unix {
    target.path = /usr/local/lib
    INSTALLS += target
}

INCLUDEPATH += $$PWD/../
