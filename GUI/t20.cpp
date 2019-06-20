#include "t20.h"
#include <QDebug>
#include <QDataStream>

T20::T20()
{
}

QByteArray T20::T20_Serialize()
{
    QByteArray ser;
    ser.append(this->startByte);
    ser.append(8);
    ser.append(this->mode);
    ser.append(this->rollTx);
    ser.append(this->pitchTx);
    ser.append(this->yawTx);
    ser.append(this->liftTx);

    this->crcCalc();

    ser.append(this->crc);

    return ser;
}

void T20::T20_Unserialize(QByteArray ser)
{
    this->startByte = ser.at(0);
    this->length    = ser.at(1);
    this->mode      = ser.at(2);

    // Get the system time from the drone in microseconds (usec)
    QByteArray st;
    st = ser.mid(3, 4);
    this->sysTime = st.toUInt();

    QByteArray valueR1;
    valueR1.append(ser.at(8)).append(ser.at(7));
    QByteArray valueR2;
    valueR2.append(ser.at(10)).append(ser.at(9));
    QByteArray valueR3;
    valueR3.append(ser.at(12)).append(ser.at(11));
    QByteArray valueR4;
    valueR4.append(ser.at(14)).append(ser.at(13));

    QDataStream(valueR1) >> this->rotor1Rx;
    QDataStream(valueR2) >> this->rotor2Rx;
    QDataStream(valueR3) >> this->rotor3Rx;
    QDataStream(valueR4) >> this->rotor4Rx;

    QByteArray valuePhi;
    valuePhi.append(ser.at(16)).append(ser.at(15));
    QByteArray valueTheta;
    valueTheta.append(ser.at(18)).append(ser.at(17));
    QByteArray valuePsi;
    valuePsi.append(ser.at(20)).append(ser.at(19));

    QDataStream(valuePhi) >> this->phi;
    QDataStream(valueTheta) >> this->theta;
    QDataStream(valuePsi) >> this->psi;

    QByteArray valueSp;
    valueSp.append(ser.at(22)).append(ser.at(21));
    QByteArray valueSq;
    valueSq.append(ser.at(24)).append(ser.at(23));
    QByteArray valueSr;
    valueSr.append(ser.at(26)).append(ser.at(25));

    QDataStream(valueSp) >> this->sp;
    QDataStream(valueSq) >> this->sq;
    QDataStream(valueSr) >> this->sr;

    QByteArray valueTemp;
    valueTemp.append(ser.at(30)).append(ser.at(29)).append(ser.at(28)).append(ser.at(27));
    QByteArray valueVoltage;
    valueVoltage.append(ser.at(32)).append(ser.at(31));
    QByteArray valuePressure;
    valuePressure.append(ser.at(36)).append(ser.at(35)).append(ser.at(34)).append(ser.at(33));

    QDataStream(valueTemp) >> this->temp;
    QDataStream(valueVoltage) >> this->voltage;
    QDataStream(valuePressure) >> this->pressure;

    this->crc = ser.at(37);
}

bool T20::T20_Verify()
{
    if(this->startByte != 0xAA){
        return false;
    }

    if(this->length != 0x08 && this->length != 0x26){
        return false;
    }

    return true;

    // CRC Check here!!!!!!

    //uint8_t* crcCheck = this->crcCalc();

//    if(this->crc == *crcCheck){
//        qDebug("SUCCESS: Packet received");
//        return true;
//    } else {
//        qDebug("ERROR: CRC failed");
//        return true;
//    }

}

void *T20::crcCalc () {
    // Initialize sum
//    static uint8_t crcSum = 0;

//    crcSum ^= this->getStartByte();
//    crcSum ^= this->getLength();
//    crcSum ^= this->getMode();

//    if(this->getLength() == 0x15){
//        crcSum ^= this->getTimestamp();
//    }

//    crcSum ^= this->getRoll();
//    crcSum ^= this->getPitch();
//    crcSum ^= this->getYaw();
//    crcSum ^= this->getLift();

//    // If packet length is 21 bytes then it is a received packet
//    // and we should take into account more fields

//    if(this->getLength() == 0x15){

//        crcSum ^= this->getPhi();
//        crcSum ^= this->getPsi();
//        crcSum ^= this->getTheta();

//        crcSum ^= this->getSp();
//        crcSum ^= this->getSq();
//        crcSum ^= this->getSr();

//        crcSum ^= this->getVoltage();
//        crcSum ^= this->getPressure();
//        crcSum ^= this->getTemperature();

//        return &crcSum;

//    } else {
//        // Set CRC only for the 8-byte packets, because they are the ones we send
//        this->setCRC(crcSum);
//        return nullptr;
//    }


    this->crc = 0;

    this->crc ^= this->getStartByte();
    this->crc ^= this->getLength();
    this->crc ^= this->getMode();
    this->crc ^= this->rollTx;
    this->crc ^= this->pitchTx;
    this->crc ^= this->yawTx;
    this->crc ^= this->liftTx;

    //return &crcSum;

}
