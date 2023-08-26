#pragma once
#include "mbed.h"

enum CONTROLLER {
    UP, RIGHT, DOWN, LEFT,
    TRIANGLE, CIRCLE, CROSS, SQUARE,
    L1, R1,
    SHARE, OPTIONS,PS,TOUCHPAD,
    L3, R3,
    CONNECTION,
    LeftHatX, LeftHatY,
    RightHatX, RightHatY,
    L2, R2,
};
enum TP {
    X, Y,
};

class PS4
{
private:
    char data[17];
    int DATA_CONTROLLER_OLD[23];
    I2C *i2c;
    char ADDR;
public:
    int DATA_CONTROLLER[23];
    int DATA_TP[2][2];
    PS4(I2C *i, char addr);
    bool UpDate();
    int getPress( CONTROLLER buttom );
    int getClick( CONTROLLER button, bool mode );
    int touchPad( TP direction, bool finger );
};