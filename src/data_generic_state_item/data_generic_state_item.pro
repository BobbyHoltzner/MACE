#-------------------------------------------------
#
# Project created by QtCreator 2017-02-03T09:08:42
#
#-------------------------------------------------

QT       -= core gui

TARGET = data_generic_state_item
TEMPLATE = lib

DEFINES += DATA_GENERIC_STATE_ITEM_LIBRARY

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
    state_attitude.cpp \
    state_global_position.cpp \
    state_local_position.cpp \
    state_global_velocity.cpp \
    state_local_velocity.cpp \
    state_generic_position.cpp \
    state_global_position_ex.cpp \
    state_local_position_ex.cpp \
    state_airspeed.cpp

HEADERS +=\
        data_generic_state_item_global.h \
    state_attitude.h \
    state_global_position.h \
    state_local_position.h \
    state_global_velocity.h \
    state_local_velocity.h \
    state_item_components.h \
    state_generic_position.h \
    state_global_position_ex.h \
    state_local_position_ex.h \
    state_airspeed.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data_generic_state_item.lib release/data_generic_state_item.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data_generic_state_item.lib debug/data_generic_state_item.dll
INSTALLS += lib


#Header file copy
headers.path    = $$(MACE_ROOT)/include/data_generic_state_item
headers.files   += $$HEADERS
INSTALLS       += headers

INCLUDEPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

unix{
    EigenInclude = $$system(pkg-config --cflags eigen3)
    EigenInclude = $$replace(EigenInclude, "-I", "")/eigen3
    INCLUDEPATH += $$EigenInclude
}
win32{
    INCLUDEPATH += "C:\Program Files (x86)\Eigen\include\eigen3"
}

