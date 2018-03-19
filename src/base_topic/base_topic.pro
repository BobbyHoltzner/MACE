#-------------------------------------------------
#
# Project created by QtCreator 2017-11-02T19:48:03
#
#-------------------------------------------------

QT       -= core gui

TARGET = base_topic
TEMPLATE = lib

DEFINES += BASE_TOPIC_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    geometry/line_2DC_topic.cpp \
    pose/cartesian_2D_topic.cpp \
    vehicle_topics.cpp

HEADERS += \
    base_topic_global.h \
    geometry/line_2DC_topic.h \
    pose/cartesian_2D_topic.h \
    base_topic_components.h \
    vehicle_topics.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/base_topic.lib release/base_topic.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/base_topic.lib debug/base_topic.dll
INSTALLS += lib


#Header file copy
headers.path    = $$(MACE_ROOT)/include/base_topic
headers.files   += \
    base_topic_global.h \
    base_topic_components.h
INSTALLS       += headers

#Header file copy
headers_geometry.path    = $$(MACE_ROOT)/include/base_topic/geometry
headers_geometry.files   += \
    geometry/line_2DC_topic.h
INSTALLS       += headers_geometry

#Header file copy
headers_pose.path    = $$(MACE_ROOT)/include/base_topic/pose
headers_pose.files   += \
    pose/cartesian_2D_topic.h
INSTALLS       += headers_pose

INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase
