#include "PS4.h"

PS4::PS4(I2C *i, char addr)
{
    i2c = i;
    ADDR = addr;

    for(int i=0; i<23; i++) {
        DATA_CONTROLLER[i] = 0;
        DATA_CONTROLLER_OLD[i] = 0;
    }
}

bool PS4::UpDate()
{
    //Make backup
    memcpy( DATA_CONTROLLER_OLD, DATA_CONTROLLER, sizeof(DATA_CONTROLLER) );

    //Get data by I2C
    int s = i2c->read((ADDR | 1), data, 17);

    if(s == 0) {

        //UpDate
        DATA_CONTROLLER[CONNECTION] = (data[0] >> 0) & 0x01;
        DATA_CONTROLLER[UP] = (data[1] >> 7) & 0x01;
        DATA_CONTROLLER[RIGHT] = (data[1] >> 6) & 0x01;
        DATA_CONTROLLER[DOWN] = (data[1] >> 5) & 0x01;
        DATA_CONTROLLER[LEFT] = (data[1] >> 4) & 0x01;
        DATA_CONTROLLER[TRIANGLE] = (data[1] >> 3) & 0x01;
        DATA_CONTROLLER[CIRCLE] = (data[1] >> 2) & 0x01;
        DATA_CONTROLLER[CROSS] = (data[1] >> 1) & 0x01;
        DATA_CONTROLLER[SQUARE] = (data[1] >> 0) & 0x01;
        DATA_CONTROLLER[L1] = (data[2] >> 7) & 0x01;
        DATA_CONTROLLER[R1] = (data[2] >> 6) & 0x01;
        DATA_CONTROLLER[SHARE] = (data[2] >> 5) & 0x01;
        DATA_CONTROLLER[OPTIONS] = (data[2] >> 4) & 0x01;
        DATA_CONTROLLER[L3] = (data[2] >> 3) & 0x01;
        DATA_CONTROLLER[R3] = (data[2] >> 2) & 0x01;
        DATA_CONTROLLER[PS] = (data[2] >> 1) & 0x01;
        DATA_CONTROLLER[TOUCHPAD] = (data[2] >> 0) & 0x01;
        DATA_CONTROLLER[LeftHatX] = data[3];
        DATA_CONTROLLER[LeftHatY] = data[4];
        DATA_CONTROLLER[RightHatX] = data[5];
        DATA_CONTROLLER[RightHatY] = data[6];
        DATA_CONTROLLER[L2] = data[7];
        DATA_CONTROLLER[R2] = data[8];
        DATA_TP[X][0] = data[9] >> 8 + data[10];
        DATA_TP[Y][0] = data[11] >> 8 + data[12];
        DATA_TP[X][1] = data[13] >> 8 + data[14];
        DATA_TP[Y][1] = data[15] >> 8 + data[16];
        
    } else {
        for(int i = 0; i < 23; i++) {
            DATA_CONTROLLER[i] = i >= 15 && i <= 18 ? 127 : 0;
        }
    }
    return s;
}

int PS4::getPress( CONTROLLER buttom )
{
    if(buttom < 23) {
        return DATA_CONTROLLER[buttom];
    } else {
        return 0;
    }
}

int PS4::getClick( CONTROLLER button, bool mode )
{
    //mode:0 -> Rising
    //mode:1 -> Falling

    if(button <= 13) {
        return mode*(DATA_CONTROLLER[button]*!DATA_CONTROLLER_OLD[button])
               + !mode*(!DATA_CONTROLLER[button]*DATA_CONTROLLER_OLD[button]);

    } else if( (19<=button) && (button<=20) ) {
        return mode*((DATA_CONTROLLER[button]>127)*!(DATA_CONTROLLER_OLD[button]>127))
               + !mode*(!(DATA_CONTROLLER[button]>127)*(DATA_CONTROLLER_OLD[button]>127));

    } else {
        return 0;
    }
}

int PS4::touchPad( TP direction, bool finger )
{
    return DATA_TP[direction][finger];
}