#include "cockpit.h"
#include "ui_cockpit.h"

#include "serialportreader.h"
#include "t20.h"

#include <QSerialPortInfo>
#include <QDebug>
#include <QComboBox>
#include <QJoysticks.h>

CockPit::CockPit(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CockPit)
{

    ui->setupUi(this);

    connect(ui->buttonGroup, SIGNAL(buttonClicked(int)), this, SLOT(switchGraph(int)));

    // Create chart
    createChart();

    // Enumerate serial devices
    SerialEnumerator();

    // Create timer for periodic send of packets
    QTimer* period = new QTimer(this);
    period->start(20);

    // Create periodic packet send event
    connect(period, SIGNAL(timeout()), this, SLOT(transmitPacket()));

    // Instantiate joystick driver
    QJoysticks* instance = QJoysticks::getInstance();

    /* Enable the virtual joystick */
    instance->setVirtualJoystickRange (1);
    instance->setVirtualJoystickEnabled (true);

    // Connect the instance with the main app object
    connect(instance, SIGNAL(axisChanged(int, int, qreal)), this, SLOT(handleJoystick(int, int, qreal)));

    // Initialize txPacket
    txPacket = new (T20);
}

CockPit::~CockPit()
{
    delete ui;
}

void CockPit::SerialEnumerator() {
    const auto infos = QSerialPortInfo::availablePorts();

    for (const QSerialPortInfo &info : infos) {

        ui->comboBox->addItem(info.portName());

    }

}

// Connect to serial port

void CockPit::on_pushButton_clicked()
{

    // Get the port that user has selected

    // Define serial port to listen to
    QString serialPortName = ui->comboBox->currentText();

    // Set port properties
    serialPort.setPortName(serialPortName);
    serialPort.setBaudRate(QSerialPort::Baud115200);

    // Set status text
    QString statusText;

    if (!serialPort.open(QIODevice::ReadWrite)) {

        statusText = QString("Failed to open port %1, error: %2").arg(serialPortName).arg(serialPort.errorString());

    } else {

        // Notify user that has connected successfully and grey out connect
        // button to avoid clicking on it again

        statusText = "Connected";
        ui->pushButton->setEnabled(false);

        // Create a new object to read data from the serial port
        serialPortReader = new SerialPortReader(&serialPort);

        // Put the incoming data to GUI elements
        connect(serialPortReader, SIGNAL(packetToGui(T20*)), this, SLOT(writeToGui(T20*)));

    }

    ui->status_lbl->setText(statusText);

}

// Writes the received data to the appropriate places in the GUI

void CockPit::writeToGui(T20* packet)
{
    ui->pitch_graph->setText(QString::number((int)packet->getRotor1()));
    ui->roll_graph->setText(QString::number((int)packet->getRotor2()));
    ui->yaw_graph->setText(QString::number((int)packet->getRotor3()));
    ui->lift_graph->setText(QString::number((int)packet->getRotor4()));
    ui->temp_data->setText(QString::number((int)packet->getTemperature()));
    ui->pres_data->setText(QString::number((int)packet->getPressure()));
    ui->batteryBar->setValue((int)packet->getVoltage());
    ui->psi_value->setText(QString::number((int)packet->getPsi()));
    ui->phi_value->setText(QString::number((int)packet->getPhi()));
    ui->theta_value->setText(QString::number((int)packet->getTheta()));
    ui->sp_value->setText(QString::number((int)packet->getSp()));
    ui->sq_value->setText(QString::number((int)packet->getSq()));
    ui->sr_value->setText(QString::number((int)packet->getSr()));


    // Update the datapoint, thus the chart series

    switch (currentInput) {
    case Temperature:
        datapoint = packet->getTemperature();
        break;
    case Pressure:
        datapoint = packet->getPressure();
        break;
    case Rotor1:
        datapoint = packet->getRotor1();
        break;
    case Rotor2:
        datapoint = packet->getRotor2();
        break;
    case Rotor3:
        datapoint = packet->getRotor3();
        break;
    case Rotor4:
        datapoint = packet->getRotor4();
        break;
    }
}

// Handles the joystick axes values and puts them on the packet fields

void CockPit::handleJoystick(int j, int axis, qreal value)
{
    j = 0;
    int txVal = static_cast<int>(value);

    switch (axis) {
    case 0:
        txPacket->setPitch(txVal);
        // Write also to GUI
        break;
    case 1:
        txPacket->setRoll(txVal);
        break;
    case 2:
        txPacket->setYaw(txVal);
        break;
    case 3:
        txPacket->setLift(txVal);
        break;
    }

    txPacket->setMode(2);

}

// Actually transmits the packet

void CockPit::transmitPacket()
{
    if(this->serialPort.isOpen()){
        // Serialize the packet into bytes to be transmitted over serial
        QByteArray txBytes = txPacket->T20_Serialize();

        // Transmit the bytes
        this->serialPort.write(txBytes);
    }
}

// Creates the chart

void CockPit::createChart()
{
    chart = new QChart();
    realtimeSeries = new QLineSeries();
    chart->setTitle(chartTitle);
    chart->legend()->hide();

    chart->addSeries(realtimeSeries);
    realtimeSeries->setColor(QColor::fromRgb(140,198,62));
    // Create axis of type datetime for x-axis
    axisx = new QDateTimeAxis;
    chart->addAxis(axisx, Qt::AlignBottom);

    // Create axis of type value for y-axis
    axisy = new QValueAxis;
    chart->addAxis(axisy, Qt::AlignLeft);
    chart->axisY()->setRange(0, 400);

    // Create DAQ Timer
    QTimer* realtimeTimer = new QTimer(this);
    connect(realtimeTimer, SIGNAL(timeout()), this, SLOT(rtTimerTimeout()));
    realtimeTimer->start(1000);

    // Add the created chart to it's container

    chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    chartView->setParent(this);
    chartView->setGeometry(35, 145, 400, 316);

}

void CockPit::setGraphInputField(CockPit::ChartInputs input)
{
    this->currentInput = input;
}

void CockPit::rtTimerTimeout()
{
    QDateTime now = QDateTime::currentDateTime();

    realtimeSeries->append(now.toSecsSinceEpoch(), datapoint);

    chart->removeSeries(realtimeSeries);

    if(realtimeSeries->count() > 60) {
        realtimeSeries->removePoints(0, 1);
    }

    chart->addSeries(realtimeSeries);
    chartView->repaint();
}

// Switch graph depending on which button was clicked

void CockPit::switchGraph(int id){

    chart->removeSeries(realtimeSeries);

    switch (id) {
    case -2:
        setGraphInputField(CockPit::ChartInputs::Temperature);
        chartTitle = "Temperature";
        break;
    case -3:
        setGraphInputField(CockPit::ChartInputs::Pressure);
        chartTitle = "Pressure";
        break;
    case -4:
        setGraphInputField(CockPit::ChartInputs::Rotor1);
        chartTitle = "Rotor 1 Speed";
        break;
    case -5:
        setGraphInputField(CockPit::ChartInputs::Rotor2);
        chartTitle = "Rotor 2 Speed";
        break;
    case -6:
        setGraphInputField(CockPit::ChartInputs::Rotor3);
        chartTitle = "Rotor 3 Speed";
        break;
    case -7:
        setGraphInputField(CockPit::ChartInputs::Rotor4);
        chartTitle = "Rotor 4 Speed";
        break;
    }
    chart->addSeries(realtimeSeries);
    chartView->repaint();
}

// Handles the the keyboard events

void CockPit::keyReleaseEvent( QKeyEvent* event ) {
    switch ( event->key() ) {

    // Mode keys

    case Qt::Key_0:
        txPacket->setMode(SAFE_MODE);
        break;
    case Qt::Key_1:
        txPacket->setMode(PANIC_MODE);
        break;
    case Qt::Key_2:
        txPacket->setMode(MANUAL_MODE);
        break;
    case Qt::Key_3:
        txPacket->setMode(CALIBRATION_MODE);
        break;
    case Qt::Key_4:
        txPacket->setMode(YAW_MODE);
        break;
    case Qt::Key_5:
        txPacket->setMode(FULL_MODE);
        break;
    case Qt::Key_6:
        txPacket->setMode(RAW_MODE);
        break;
    case Qt::Key_7:
        txPacket->setMode(HEIGHT_MODE);
        break;
    case Qt::Key_8:
        txPacket->setMode(WIRELESS_MODE);
        break;

        // Rotor keys
    case Qt::Key_A:
        txPacket->lift_offset = checkByteOverflow(txPacket->lift_offset, 1);
        break;
    case Qt::Key_Z:
        txPacket->lift_offset = checkByteOverflow(txPacket->lift_offset, -1);
        break;
    case Qt::Key_W: //yaw rate up
        txPacket->yaw_offset = checkByteOverflow(txPacket->yaw_offset, 1);
        break;
    case Qt::Key_Q: //yaw rate down
        txPacket->yaw_offset = checkByteOverflow(txPacket->yaw_offset, -1);
        break;
    case Qt::Key_U: //set P of yaw up
        txPacket->P = 2;
        break;
    case Qt::Key_J: //set P of yaw down
        txPacket->P = 1;
        break;
    case Qt::Key_I: //set roll/pitch P1 up
        txPacket->P1 = 2;
        break;
    case Qt::Key_K: //set roll/pitch P1 down
        txPacket->P1 = 1;
        break;
    case Qt::Key_O: //set roll/pitch P2 up
        txPacket->P2 = 2;
        break;
    case Qt::Key_L: //set roll/pitch P2 down
        txPacket->P2 = 1;
        break;

    default:
        event->ignore();
        break;
    }

}

// Check overflows

int8_t CockPit::checkByteOverflow(int8_t value, int8_t offset) {

    // Make sure overflows won't happen.

    int8_t txValue;

    if((int16_t)(value + offset) > 127) {
        txValue = 127;
    } else if ((int16_t)(value + offset) < -128) {
        txValue = -128;
    } else {
        txValue = value + offset;
    }

    return txValue;
}

uint8_t CockPit::checkByteOverflowLift(int8_t value, int8_t offset) {

    // Make sure overflows won't happen.

    uint8_t txValue;

    if((int16_t)(value + offset +127) > 255) {
        txValue = 255;
    } else if ((int16_t)(value + offset + 127) < 0) {
        txValue = 0;
    } else {
        txValue = value + offset + 127;
    }

    return txValue;
}
