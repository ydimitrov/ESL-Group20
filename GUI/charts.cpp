#include "chartview.h"
#include "ui_chartview.h"

#define margin 10
#define xBasicTileSize 102
#define yBasicTileSize 90

ChartView::ChartView(QWidget *parent) :
    TileInterface(parent),
    ui(new Ui::ChartView)
{
    ui->setupUi(this);
}

ChartView::ChartView(QString _deviceid, QString _inputid, QWidget *parent) :
    TileInterface(parent),
    ui(new Ui::ChartView),
    deviceid(_deviceid),
    inputid(_inputid)
{
    ui->setupUi(this);

    currPeriod = period_rt;
    minValueRT = 0.0;
    maxValueRT = 1.0;
}


ChartView::~ChartView()
{
    delete ui;
}


void ChartView::createChart()
{

    newChart = new QChart();
    newChart->setTitle(name);
    newChart->legend()->hide();
    newChart->acceptTouchEvents();

    // Create axis of type datetime for x-axis
    axisx = new QDateTimeAxis;
    newChart->addAxis(axisx, Qt::AlignBottom);

    // Create axis of type value for y-axis
    axisy = new QValueAxis;
    newChart->addAxis(axisy, Qt::AlignLeft);

    QFont axisfont;
    axisfont.setPointSize(12);
    axisfont.setFamily("Calibri");
    axisfont.setBold(false);

    axisy->setTitleFont(axisfont);
    axisy->setTitleText(getUnit());

    // Create a series that can hold historical measurement data. The series will be updated every time new data is plotted
    // historicalSeries = new QLineSeries(newChart);
    // newChart->addSeries(historicalSeries);

    // Attach the axis to the series to keep them in sync
    // historicalSeries->attachAxis(axisx);
    // historicalSeries->attachAxis(axisy);
    // historicalSeries->setColor(QColor::fromRgb(140,198,62));

    // Create a series that will hold real time measurement data. This series will get one new measurement point every second with a rotating wnidow of 5 minutes
    realtimeSeries = new QLineSeries(newChart);
    newChart->addSeries(realtimeSeries);
    realtimeSeries->setColor(QColor::fromRgb(140,198,62));

    // Attach the axis to the series to keep them in sync
    realtimeSeries->attachAxis(axisx);
    realtimeSeries->attachAxis(axisy);

    // Create DAQ Timer
    realtimeTimer = new QTimer(this);
    connect(realtimeTimer, SIGNAL(timeout()), this, SLOT(realtimeTimerTimeout()));
    realtimeTimer->start(1000);

    // Add the created chart to it's container
    newChart->setContentsMargins(-30,-30,-30,-30); // Nasty way to fix the margins between the chart container and the tile
    chartView = new QChartView(newChart);
    chartView->setRenderHint(QPainter::Antialiasing);

    ui->chartContainer->setLayout(base);

}

void ChartView::realtimeTimerTimeout()
{

    QString dataQuery = QString("SELECT data, unitid, created FROM VPTerminal.daq_data_rt WHERE created >= DATE_SUB(NOW(), INTERVAL 1 second) AND deviceid = %1 AND inputid = %2  ORDER BY created ASC LIMIT 1").arg(getDeviceId()).arg(getDevInputId());
    DBRESULT rtData = database::SqlEngine::getInstance().selectQuery(dataQuery);

    for (DBROW entry : rtData){
        double datapoint = realtimeCalculator::getInstance()->convertUnit(entry["data"].toDouble(), entry["unitid"].toInt(), getUnitId());
        QDateTime dbDateTime = QDateTime::fromString(entry["created"], "yyyy-MM-ddThh:mm:ss");
        dbDateTime.setTimeSpec(Qt::UTC);

        realtimeSeries->append(dbDateTime.toLocalTime().toMSecsSinceEpoch(), datapoint);

        if(realtimeSeries->count() == 1) {
            minValueRT = datapoint;
            maxValueRT = datapoint;
        }
        // Update min/max values for yaxis scaling
        if(datapoint < minValueRT) {
            minValueRT = datapoint;
        }
        if(datapoint > maxValueRT) {
            maxValueRT = datapoint;
        }
    }

    // If we got more then 60 datapoints (1 minute), then remove the first point.
    if(realtimeSeries->count() > 60) {
        // If min or max value is removed, find new one.
        if(realtimeSeries->at(0).toPoint().ry() == minValueRT || realtimeSeries->at(0).toPoint().ry() == maxValueRT) {
            // Asign first entry in series to new value then iterate through all entries
            minValueRT = realtimeSeries->at(0).toPoint().ry();
            maxValueRT = realtimeSeries->at(0).toPoint().ry();
            for(quint16 i = 0; i<realtimeSeries->count(); i++) {
                // Update min/max values for yaxis scaling
                if(realtimeSeries->at(i).toPoint().ry() < minValueRT) {
                    minValueRT = realtimeSeries->at(i).toPoint().ry();
                }
                if(realtimeSeries->at(i).toPoint().ry() > maxValueRT) {
                    maxValueRT = realtimeSeries->at(i).toPoint().ry();
                }
            }
        }
        realtimeSeries->removePoints(0,1);
    }

    if(currPeriod == period_rt) {
        newChart->axisY()->setRange(minValueRT, maxValueRT);
        if(realtimeSeries->count() == 0)
            newChart->axisX()->setRange(QDateTime::currentDateTime().addSecs(-300), QDateTime::currentDateTime());
        else
            newChart->axisX()->setRange(QDateTime::currentDateTime().addSecs(realtimeSeries->count()*-1), QDateTime::currentDateTime());
    }

    // Apply all updates to the chart
    newChart->update();
}
