QT += core
QT -= gui

CONFIG += c++11

TARGET = TestMaps
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

QMAKE_CXXFLAGS += -std=c++11
QMAKE_CXXFLAGS_WARN_ON += -Wno-unknown-pragmas

SOURCES += main.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS


INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$PWD/../../speedLog/
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3


# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../planners/release/ -lplanners
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../planners/debug/ -lplanners
else:unix:!macx: LIBS += -L$$OUT_PWD/../planners/ -lplanners

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../tools/flann/build/lib/release/ -lflann
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../tools/flann/build/lib/debug/ -lflann
else:unix:!macx: LIBS += -L$$PWD/../../tools/flann/build/lib/ -lflann

INCLUDEPATH += $$PWD/../../tools/flann/build
DEPENDPATH += $$PWD/../../tools/flann/build
