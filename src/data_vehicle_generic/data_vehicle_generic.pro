#-------------------------------------------------
#
# Project created by QtCreator 2017-01-10T10:45:51
#
#-------------------------------------------------

QT       -= core gui

TARGET = data_vehicle_generic
TEMPLATE = lib

DEFINES += DATA_VEHICLE_GENERIC_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += \
    local_position.cpp \
    local_velocity.cpp \
    global_position.cpp \
    global_velocity.cpp

HEADERS +=\
        data_vehicle_generic_global.h \
    local_position.h \
    local_velocity.h \
    global_position.h \
    i_vehicle_topic_component.h \
    global_velocity.h \
    coordinate_frame.h




# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/module_vehicle_MAVLINK.lib release/module_vehicle_MAVLINK.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/module_vehicle_MAVLINK.lib debug/module_vehicle_MAVLINK.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/module_vehicle_MAVLINK
headers.files   += \
        module_vehicle_mavlink.h \
        module_vehicle_mavlink_global.h \
        generic_message_definition_mavlink.h
INSTALLS       += headers





INCLUDEPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core


