#-------------------------------------------------
#
# Project created by QtCreator 2017-02-02T21:35:48
#
#-------------------------------------------------

QT       -= core gui

TARGET = data_generic_state_topics
TEMPLATE = lib

DEFINES += DATA_GENERIC_STATE_TOPICS_LIBRARY

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
    attitude_topic.cpp \
    global_position_topic.cpp \
    global_velocity_topic.cpp \
    local_position_topic.cpp \
    local_velocity_topic.cpp

HEADERS +=\
        data_generic_state_topics_global.h \
    attitude_topic.h \
    global_position_topic.h \
    global_velocity_topic.h \
    local_position_topic.h \
    local_velocity_topic.h \
    components.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data_generic_state_topics.lib release/data_generic_state_topics.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data_generic_state_topics.lib debug/data_generic_state_topics.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/data_generic_state_topics
headers.files   += \
        data_generic_state_topics_global.h \
    attitude_topic.h \
    global_position_topic.h \
    global_velocity_topic.h \
    local_position_topic.h \
    local_velocity_topic.h
INSTALLS       += headers


INCLUDEPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix:!macx: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_items/release/ -ldata_generic_state_items
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_items/debug/ -ldata_generic_state_items
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_state_items/ -ldata_generic_state_items
