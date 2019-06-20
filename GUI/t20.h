#ifndef T20_H
#define T20_H

#include <QObject>

class T20
{
public:
    T20();

    // Set packet methods

    QByteArray T20_Serialize();
    void T20_Unserialize(QByteArray);
    bool T20_Verify();

    void setRoll(int8_t _roll){ this->rollTx = _roll; }
    void setPitch(int8_t _pitch){ this->pitchTx = _pitch; }
    void setYaw(int8_t _yaw){ this->yawTx = _yaw; }
    void setLift(uint8_t _lift){ this->liftTx = _lift; }
    void setMode(uint8_t _mode){ this->mode = _mode; }
    void setSysTime(uint8_t _st){ this->sysTime = _st; }
    void setCRC(uint8_t _crc){this->crc = _crc;}

    uint8_t getStartByte(){return this->startByte;}
    uint8_t getLength(){return this->length;}
    uint8_t getMode(){return this->mode;}

    uint32_t getTimestamp(){return this->sysTime;}

    uint16_t getRotor1(){return this->rotor1Rx;}
    uint16_t getRotor2(){return this->rotor2Rx;}
    uint16_t getRotor3(){return this->rotor3Rx;}
    uint16_t getRotor4(){return this->rotor4Rx;}

    uint16_t getPhi(){return this->phi;}
    uint16_t getPsi(){return this->psi;}
    uint16_t getTheta(){return this->theta;}

    uint16_t getSp(){return this->sp;}
    uint16_t getSq(){return this->sq;}
    uint16_t getSr(){return this->sr;}

    uint32_t getTemperature(){return this->temp;}
    uint16_t getVoltage(){return this->voltage;}
    uint32_t getPressure(){return this->pressure;}

    void *crcCalc();

    int8_t yaw_offset   = 0;
    int8_t roll_offset  = 0;
    int8_t pitch_offset = 0;
    int8_t lift_offset  = 0;
    int8_t P = 0;
    int8_t P1 = 0;
    int8_t P2 = 0;

private:
    // Set packet fields
    // Universal fields
    uint8_t startByte       = 0xAA;
    uint8_t length          = 0x08;
    uint8_t mode            = 0x02;
    uint8_t crc             = 0x00;


    // Tx fields
    int8_t rollTx           = 0x00;
    int8_t pitchTx          = 0x00;
    int8_t yawTx            = 0x00;
    uint8_t liftTx          = 0x00;

    // Rx fields

    uint32_t sysTime        = 0x00000000;

    uint16_t rotor1Rx        = 0x0000;
    uint16_t rotor2Rx        = 0x0000;
    uint16_t rotor3Rx        = 0x0000;
    uint16_t rotor4Rx        = 0x0000;

    uint16_t phi             = 0x0000;
    uint16_t theta           = 0x0000;
    uint16_t psi             = 0x0000;
    uint16_t sp              = 0x0000;
    uint16_t sq              = 0x0000;
    uint16_t sr              = 0x0000;
    uint32_t temp            = 0x00000000;
    uint16_t voltage         = 0x0000;
    uint32_t pressure        = 0x00000000;

    // Offsets. Not part of the final packet, but they're contributing to those values


};

#endif // T20_H
