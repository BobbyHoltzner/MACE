TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    data_interpolation.cpp

HEADERS += \
    example_module.h \
    data_interpolation.h

QMAKE_CXXFLAGS += -std=c++11

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core


EigenInclude = $$system(pkg-config --cflags eigen3)
EigenInclude = $$replace(EigenInclude, "-I", "")/eigen3
INCLUDEPATH += $$EigenInclude


INCLUDEPATH += $$PWD/../
DEPENDPATH += $$PWD/../
