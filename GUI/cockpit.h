#ifndef COCKPIT_H
#define COCKPIT_H

#include <QMainWindow>
#include <QSerialPort>
#include <QtCharts>
#include "serialportreader.h"

// Definitions
#define SAFE_MODE 0
#define PANIC_MODE 1
#define MANUAL_MODE 2
#define CALIBRATION_MODE 3
#define YAW_MODE 4
#define FULL_MODE 5
#define RAW_MODE 6
#define HEIGHT_MODE 7
#define WIRELESS_MODE 8
#define P_INCREMENT 9
#define P_DECREMENT 10
#define P1_INCREMENT 11
#define P1_DECREMENT 12
#define P2_INCREMENT 13
#define P2_DECREMENT 14

namespace Ui {
class CockPit;
}

class CockPit : public QMainWindow
{
    Q_OBJECT

public:
    explicit CockPit(QWidget *parent = 0);
    ~CockPit();
    void SerialEnumerator();

    int8_t checkByteOverflow(int8_t value, int8_t offset);
    uint8_t checkByteOverflowLift(int8_t value, int8_t offset);

protected:
    void keyReleaseEvent(QKeyEvent *event);

private slots:
    void on_pushButton_clicked();
    void switchGraph(int id);
    void writeToGui(T20*);
    void handleJoystick(int j, int axis, qreal value);
    void transmitPacket();
    void rtTimerTimeout();

private:
    Ui::CockPit *ui;
    QSerialPort serialPort;
    SerialPortReader* serialPortReader;
    T20* txPacket; // Set packet to be periodically transmitted

    enum ChartInputs {Temperature, Pressure, Rotor1, Rotor2, Rotor3, Rotor4};
    ChartInputs currentInput;
    QChart *chart;
    QChartView *chartView;
    QLineSeries* realtimeSeries;
    int32_t datapoint;
    QDateTimeAxis* axisx;
    QValueAxis* axisy;
    QString chartTitle;

    void createChart();
    void setGraphInputField(ChartInputs);
};

#endif // COCKPIT_H
