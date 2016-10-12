#-------------------------------------------------
#
# Project created by QtCreator 2016-10-04T03:38:02
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = LineFollowerSimulator
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    arduino.cpp \
    route.cpp \
    robot.cpp \
    utils.cpp \
    robotitem.cpp \
    pid.cpp \
    ../version-2/odometry.cpp \
    ../version-2/paths.cpp \
    ../version-2/version-2.cpp \
    PID_v1.cpp \
    Adafruit_MotorShield.cpp

HEADERS  += mainwindow.h \
    arduino.h \
    route.h \
    robot.h \
    utils.h \
    robotitem.h \
    pid.h \
    ../version-2/odometry.h \
    ../version-2/paths.h \
    ../version-2/version-2.ino \
    Adafruit_MotorShield.h \
    PID_v1.h

FORMS    += mainwindow.ui

CONFIG += c++11
