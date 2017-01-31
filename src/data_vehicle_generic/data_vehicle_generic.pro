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
    global_velocity.cpp \
    attitude.cpp \
    position.cpp

HEADERS +=\
        data_vehicle_generic_global.h \
    local_position.h \
    local_velocity.h \
    global_position.h \
    global_velocity.h \
    coordinate_frame.h \
    components.h \
    attitude.h \
    vehicle_types.h \
    position.h




# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data_vehicle_generic.lib release/data_vehicle_generic.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data_vehicle_generic.lib debug/data_vehicle_generic.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/data_vehicle_generic
headers.files   += \
        attitude.h \
        local_position.h \
        local_velocity.h \
        global_position.h \
        global_velocity.h \
        coordinate_frame.h \
        position.h \
        data_vehicle_generic_global.h
INSTALLS       += headers





INCLUDEPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata


