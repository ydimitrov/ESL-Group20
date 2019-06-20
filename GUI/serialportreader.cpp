/****************************************************************************
**
** Copyright (C) 2013 Laszlo Papp <lpapp@kde.org>
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtSerialPort module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "serialportreader.h"

#include <QDebug>
#include <QCoreApplication>

QT_USE_NAMESPACE



SerialPortReader::SerialPortReader(QSerialPort *serialPort, QObject *parent)
    : QObject(parent)
    , m_serialPort(serialPort)
    , m_standardOutput(stdout)
{
    connect(m_serialPort, &QSerialPort::readyRead, this, &SerialPortReader::handleReadyRead);
    connect(m_serialPort, static_cast<void (QSerialPort::*)(QSerialPort::SerialPortError)>(&QSerialPort::error),
            this, &SerialPortReader::handleError);
    connect(&m_timer, &QTimer::timeout, this, &SerialPortReader::handleTimeout);

    // Create .csv file for logging data
    QString fn = QDateTime::currentDateTimeUtc().toString() + "dronelog.csv";

    m_csv.setFileName(fn);

    if (m_csv.open(QFile::WriteOnly|QFile::Append)) {
        QTextStream stream(&m_csv);
        stream << "Timestamp" << ";"
               << "Rotor1" << ";"
               << "Rotor2" << ";"
               << "Rotor3" << ";"
               << "Rotor4" << ";"
               << "Temperature" << ";"
               << "Pressure" << ";"
               << "Voltage" << "\n";
    }
    m_csv.close();

    m_timer.start(500);
}

SerialPortReader::~SerialPortReader()
{
}

void SerialPortReader::handleReadyRead()
{
    qint64 numberOfBytes = (m_serialPort->bytesAvailable());

    // If numberOfBytes is greater than 21, then it means that we have at least one packet to receive
    if(numberOfBytes > 38 && m_serialPort->isReadable()) {

        // Read all data
        m_readData = m_serialPort->readAll();

        QString foo = QString(m_readData);

        //qDebug() << foo;
        // Search for start byte
        int i;
        for(i=0; i<numberOfBytes; i++){
            if (m_readData.at(i) == (char)0xAA){

                // Extract packets from byte stream
                // We extract 38 bytes (size of the packet)
                QByteArray cPacket;
                cPacket = m_readData.mid(i, 38);

                // Create an actual packet and verify it
                // If packet size is different than 38 bytes, drop it

                if(cPacket.size() == 38){
                    T20* packet = new(T20);
                    packet->T20_Unserialize(cPacket);

                    bool correctPacket = packet->T20_Verify();

                    if(correctPacket == true) {
                        // Data are acceptable, proceed
                        emit packetToGui(packet);

                        // Write data to .csv file [SEPARATE THREAD]


                        if (m_csv.open(QFile::WriteOnly|QFile::Append)) {
                            QTextStream stream(&m_csv);
                            stream << packet->getTimestamp() << ";"
                                   << packet->getRotor1() << ";"
                                   << packet->getRotor2() << ";"
                                   << packet->getRotor3() << ";"
                                   << packet->getRotor4() << ";"
                                   << packet->getTemperature() << ";"
                                   << packet->getPressure() << ";"
                                   << packet->getVoltage() << "\n";
                        }
                        m_csv.close();

                    } else {
                        // Drop packet
                        packet->~T20();
                    }

                } else {
                    cPacket.clear();
                }

            }
        }
    }
}

void SerialPortReader::handleTimeout()
{
    if (m_readData.isEmpty()) {
        m_standardOutput << QObject::tr("No data was currently available for reading from port %1").arg(m_serialPort->portName()) << endl;
    } else {
        //qDebug() << QString("Data successfully received from port %1").arg(m_serialPort->portName());
    }
}

void SerialPortReader::handleError(QSerialPort::SerialPortError serialPortError)
{
    if (serialPortError == QSerialPort::ReadError) {
        qDebug() << QString("An I/O error occurred while reading the data from port %1, error: %2").arg(m_serialPort->portName()).arg(m_serialPort->errorString());
        //QCoreApplication::exit(1);
    }
}
