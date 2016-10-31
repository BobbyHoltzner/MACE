#-------------------------------------------------
#
# Project created by QtCreator 2016-08-22T13:49:38
#
#-------------------------------------------------

QT       -= core gui

TARGET = mace_core
TEMPLATE = lib
win32:TARGET_EXT += .dll

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

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/mace_core.lib release/mace_core.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/mace_core.lib debug/mace_core.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/mace_core
headers.files   += $$HEADERS
INSTALLS       += headers

INCLUDEPATH += $$(MACE_ROOT)/include


unix{
    EigenInclude = $$system(pkg-config --cflags eigen3)
    EigenInclude = $$replace(EigenInclude, "-I", "")/eigen3
    INCLUDEPATH += $$EigenInclude
}
win32{
    INCLUDEPATH += "C:\Program Files (x86)\Eigen\include\eigen3"
}


INCLUDEPATH += $$PWD/../
