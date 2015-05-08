TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
QMAKE_CC= gcc -std=c99
SOURCES += main.c \
    shortest_path.c \
    map.c \
    make_decision.c

HEADERS += \
    shortest_path.h \
    map.h \
    make_decision.h

