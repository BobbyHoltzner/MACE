#-------------------------------------------------
#
# Project created by QtCreator 2017-09-16T11:10:32
#
#-------------------------------------------------

QT       -= core gui

TARGET = planners
TEMPLATE = lib

DEFINES += PLANNERS_LIBRARY

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
        planners.cpp \
    nearest_neighbor_linear.cpp \
    tsp_greedy_nearest_neighbor.cpp \
    tsp_2opt.cpp

HEADERS += \
        planners.h \
        planners_global.h \ 
    nearest_neighbor.h \
    nearest_neighbor_linear.h \
    tsp_greedy_nearest_neighbor.h \
    tsp_2opt.h

#Header file copy
headers.path    = $$(MACE_ROOT)/include/planners
headers.files   += $$HEADERS
INSTALLS       += headers

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/planners.lib release/planners.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/planners.lib debug/planners.dll
INSTALLS += lib

INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$PWD/../../speedLog/
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase
