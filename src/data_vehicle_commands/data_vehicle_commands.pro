#-------------------------------------------------
#
# Project created by QtCreator 2017-01-27T09:40:19
#
#-------------------------------------------------

QT       -= core gui

TARGET = data_vehicle_commands
TEMPLATE = lib

DEFINES += DATA_VEHICLE_COMMANDS_LIBRARY

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

SOURCES += data_vehicle_commands.cpp \
    actionCommandComponents/command_vehicle_mode.cpp \
    actionCommandComponents/command_vehicle_takeoff.cpp \
    actionCommandComponents/command_vehicle_land.cpp \
    general_mission_item.cpp \
    mission_item_topic.cpp

HEADERS += data_vehicle_commands.h\
        data_vehicle_commands_global.h \
    command_types.h \
    missionCommandComponents/command_typesN.h \
    actionCommandComponents/command_vehicle_mode.h \
    action_command_components.h \
    actionCommandComponents/command_vehicle_takeoff.h \
    actionCommandComponents/command_vehicle_land.h \
    general_mission_item.h \
    mission_item_types.h \
    mission_item_topic.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data_vehicle_commands.lib release/data_vehicle_commands.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data_vehicle_commands.lib debug/data_vehicle_commands.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/data_vehicle_commands
headers.files   += \
        action_command_components.h \
        command_types.h \
        data_vehicle_commands.h \
        data_vehicle_commands_global.h
INSTALLS       += headers

#Header file copy
headers_actionComponents.path    = $$(MACE_ROOT)/include/data_vehicle_commands/actionCommandComponents
headers_actionComponents.files   += \
        actionCommandComponents/command_vehicle_mode.h \
        actionCommandComponents/command_vehicle_takeoff.h \
        actionCommandComponents/command_vehicle_land.h


INSTALLS       += headers_actionComponents

#Header file copy
headers_missionComponents.path    = $$(MACE_ROOT)/include/data_vehicle_commands/missionCommandComponents
headers_missionComponents.files   += \

INSTALLS       += headers_missionComponents

INCLUDEPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_generic/release/ -ldata_vehicle_generic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_generic/debug/ -ldata_vehicle_generic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_vehicle_generic/ -ldata_vehicle_generic
